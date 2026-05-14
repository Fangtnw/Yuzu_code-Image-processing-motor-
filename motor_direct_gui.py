#!/usr/bin/env python3
"""
Standalone Windows-friendly GUI for one DR28T axis over USB serial.

This bypasses ROS2 and talks directly to the AZD-KEP driver using
``motor_direct.py``. It is intended for the current non-ROS workflow.
"""

from __future__ import annotations

import threading
import tkinter as tk
from tkinter import messagebox, ttk

import serial.tools.list_ports

from motor_direct import (
    DEFAULT_ACCEL_MMS2,
    DEFAULT_DECEL_MMS2,
    MAX_POS_MM,
    MAX_SPD_MMS,
    MotorDriver,
    _find_port,
    _travel_time_s,
)


STATUS_COLORS = {
    "disconnected": "#9e9e9e",
    "idle": "#4caf50",
    "busy": "#ff9800",
    "running": "#2196f3",
    "error": "#f44336",
}


class DirectMotorGui:
    def __init__(self) -> None:
        self._driver: MotorDriver | None = None
        self._worker: threading.Thread | None = None
        self._last_pos_mm = 0.0

        self._root = tk.Tk()
        self._root.title("DR28T Direct Motor GUI")
        self._root.resizable(False, False)

        self._status_dot = None
        self._status_lbl = None
        self._log = None

        self._port_var = tk.StringVar(value=_find_port() or "")
        self._baud_var = tk.StringVar(value="19200")
        self._pos_var = tk.StringVar(value="10.0")
        self._spd_var = tk.StringVar(value="5.0")
        self._accel_var = tk.StringVar(value=f"{DEFAULT_ACCEL_MMS2:.2f}")
        self._decel_var = tk.StringVar(value=f"{DEFAULT_DECEL_MMS2:.2f}")
        self._time_var = tk.StringVar(value="")

        self._build_ui()
        self._set_status("disconnected")
        self._log_print("Direct USB GUI ready.")
        self._log_print("This tool controls one DR28T axis without ROS2.")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self._root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")

        header = tk.Frame(outer, bg="#212121", pady=8)
        header.grid(row=0, column=0, sticky="ew")
        tk.Label(header, text="Status:", bg="#212121", fg="white",
                 font=("Helvetica", 10)).pack(side="left", padx=(12, 6))
        self._status_dot = tk.Label(header, text="●", bg="#212121",
                                    fg=STATUS_COLORS["disconnected"], font=("Helvetica", 14))
        self._status_dot.pack(side="left")
        self._status_lbl = tk.Label(header, text="disconnected", bg="#212121", fg="white",
                                    font=("Helvetica", 10, "bold"), width=14, anchor="w")
        self._status_lbl.pack(side="left", padx=(4, 0))

        conn = ttk.LabelFrame(outer, text="Connection", padding=12)
        conn.grid(row=1, column=0, sticky="ew", pady=(10, 6))
        conn.columnconfigure(1, weight=1)

        tk.Label(conn, text="Port").grid(row=0, column=0, sticky="w", pady=4)
        self._port_combo = ttk.Combobox(conn, textvariable=self._port_var, width=18)
        self._port_combo.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(conn, text="Connect", command=self._connect_clicked).grid(row=0, column=3, padx=(0, 6))
        ttk.Button(conn, text="Disconnect", command=self._disconnect_clicked).grid(row=0, column=4)

        tk.Label(conn, text="Baud").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Entry(conn, textvariable=self._baud_var, width=10).grid(row=1, column=1, sticky="w", padx=(8, 0))
        tk.Label(conn, text="Default: 19200", fg="gray").grid(row=1, column=2, columnspan=3, sticky="w", padx=(8, 0))

        motion = ttk.LabelFrame(outer, text="Motion Command", padding=12)
        motion.grid(row=2, column=0, sticky="ew", pady=6)

        tk.Label(motion, text="Position (mm)").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Entry(motion, textvariable=self._pos_var, width=10).grid(row=0, column=1, sticky="w", padx=(8, 0))
        tk.Label(motion, text=f"[0 - {MAX_POS_MM}]", fg="gray").grid(row=0, column=2, sticky="w", padx=(6, 0))

        tk.Label(motion, text="Speed (mm/s)").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Entry(motion, textvariable=self._spd_var, width=10).grid(row=1, column=1, sticky="w", padx=(8, 0))
        tk.Label(motion, text=f"> 0, <= {MAX_SPD_MMS}", fg="gray").grid(row=1, column=2, sticky="w", padx=(6, 0))

        tk.Label(motion, text="Accel (mm/s²)").grid(row=2, column=0, sticky="w", pady=4)
        ttk.Entry(motion, textvariable=self._accel_var, width=10).grid(row=2, column=1, sticky="w", padx=(8, 0))
        tk.Label(motion, text="Hardware ramp", fg="gray").grid(row=2, column=2, sticky="w", padx=(6, 0))

        tk.Label(motion, text="Decel (mm/s²)").grid(row=3, column=0, sticky="w", pady=4)
        ttk.Entry(motion, textvariable=self._decel_var, width=10).grid(row=3, column=1, sticky="w", padx=(8, 0))
        tk.Label(motion, text="Hardware ramp", fg="gray").grid(row=3, column=2, sticky="w", padx=(6, 0))

        tk.Label(motion, text="Wait (s)").grid(row=4, column=0, sticky="w", pady=4)
        ttk.Entry(motion, textvariable=self._time_var, width=10).grid(row=4, column=1, sticky="w", padx=(8, 0))
        tk.Label(motion, text="blank = return immediately", fg="gray").grid(row=4, column=2, sticky="w", padx=(6, 0))

        btns = tk.Frame(motion)
        btns.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        ttk.Button(btns, text="Send Command", command=self._send_clicked).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(btns, text="Go Home", command=self._home_clicked).pack(side="left", expand=True, fill="x")

        log_frame = ttk.LabelFrame(outer, text="Log", padding=8)
        log_frame.grid(row=3, column=0, sticky="nsew", pady=(6, 0))
        self._log = tk.Text(log_frame, height=10, width=64, state="disabled",
                            bg="#1e1e1e", fg="#cccccc", font=("Courier", 9),
                            relief="flat", wrap="word")
        sb = ttk.Scrollbar(log_frame, command=self._log.yview)
        self._log.configure(yscrollcommand=sb.set)
        self._log.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        self._refresh_ports()

    def _refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_combo["values"] = ports
        if not self._port_var.get() and ports:
            self._port_var.set(ports[0])

    def _log_print(self, text: str) -> None:
        self._log.config(state="normal")
        self._log.insert("end", text + "\n")
        self._log.see("end")
        self._log.config(state="disabled")

    def _set_status(self, status: str) -> None:
        self._status_dot.config(fg=STATUS_COLORS.get(status, "white"))
        self._status_lbl.config(text=status)

    def _set_status_async(self, status: str) -> None:
        self._root.after(0, lambda: self._set_status(status))

    def _ensure_driver(self) -> MotorDriver:
        if self._driver is not None:
            return self._driver

        port = self._port_var.get().strip() or _find_port()
        if not port:
            raise RuntimeError("No serial port selected or detected.")
        try:
            baud = int(self._baud_var.get().strip())
        except ValueError as exc:
            raise RuntimeError("Baud must be an integer.") from exc

        self._driver = MotorDriver.open(port, baud)
        self._log_print(f"<< connected to {port} @ {baud}")
        self._set_status_async("idle")
        return self._driver

    def _disconnect(self) -> None:
        driver = self._driver
        self._driver = None
        if driver is not None:
            try:
                driver.close()
            except Exception as exc:
                self._log_print(f"<< disconnect warning: {exc}")
        self._set_status_async("disconnected")

    def _connect_clicked(self) -> None:
        try:
            self._ensure_driver()
        except Exception as exc:
            self._set_status("error")
            messagebox.showerror("Connection Error", str(exc))
            self._log_print(f"<< connect failed: {exc}")

    def _disconnect_clicked(self) -> None:
        self._disconnect()
        self._log_print("<< disconnected")

    def _send_clicked(self) -> None:
        try:
            pos = float(self._pos_var.get().strip())
            spd = float(self._spd_var.get().strip())
            accel = float(self._accel_var.get().strip())
            decel = float(self._decel_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Input", "Position, speed, accel, and decel must be numbers.")
            return

        wait_text = self._time_var.get().strip()
        wait_s = None
        if wait_text:
            try:
                wait_s = float(wait_text)
            except ValueError:
                messagebox.showerror("Invalid Input", "Wait time must be a number.")
                return

        if not (0.0 <= pos <= MAX_POS_MM):
            messagebox.showerror("Out of Range", f"Position must be 0 - {MAX_POS_MM} mm.")
            return
        if not (0.0 < spd <= MAX_SPD_MMS):
            messagebox.showerror("Out of Range", f"Speed must be > 0 and <= {MAX_SPD_MMS} mm/s.")
            return
        if accel <= 0.0 or decel <= 0.0:
            messagebox.showerror("Out of Range", "Accel and decel must be > 0.")
            return
        if wait_s is not None and wait_s <= 0.0:
            messagebox.showerror("Out of Range", "Wait time must be > 0.")
            return
        if self._worker and self._worker.is_alive():
            messagebox.showwarning("Busy", "A command is already running.")
            return

        distance = abs(pos - self._last_pos_mm)
        est_s = _travel_time_s(distance, spd, accel)
        self._log_print(
            f">> move  pos={pos:.2f} mm  speed={spd:.2f} mm/s  "
            f"accel={accel:.2f} mm/s²  decel={decel:.2f} mm/s²"
            + (f"  wait={wait_s:.2f} s" if wait_s is not None else "")
        )
        self._worker = threading.Thread(
            target=self._run_move,
            args=(pos, spd, accel, decel, wait_s, est_s),
            daemon=True,
        )
        self._worker.start()

    def _run_move(self, pos: float, spd: float, accel: float, decel: float,
                  wait_s: float | None, est_s: float) -> None:
        try:
            driver = self._ensure_driver()
            self._set_status_async("busy")
            driver.move(pos, spd, wait_s=wait_s, accel_mms2=accel, decel_mms2=decel)
            self._last_pos_mm = pos
            self._set_status_async("running")
            self._root.after(0, lambda: self._log_print(f"<< est. travel time {est_s:.2f} s"))
            if wait_s is None:
                self._root.after(max(1, int(est_s * 1000)), lambda: self._set_status("idle"))
            else:
                self._set_status_async("idle")
        except Exception as exc:
            self._root.after(0, lambda: self._handle_async_error(f"Move failed: {exc}"))

    def _home_clicked(self) -> None:
        if self._worker and self._worker.is_alive():
            messagebox.showwarning("Busy", "A command is already running.")
            return
        self._log_print(">> home")
        self._worker = threading.Thread(target=self._run_home, daemon=True)
        self._worker.start()

    def _run_home(self) -> None:
        try:
            driver = self._ensure_driver()
            self._set_status_async("busy")
            driver.home()
            self._last_pos_mm = 0.0
            self._set_status_async("running")
            self._root.after(2000, lambda: self._set_status("idle"))
        except Exception as exc:
            self._root.after(0, lambda: self._handle_async_error(f"Home failed: {exc}"))

    def _handle_async_error(self, message: str) -> None:
        self._set_status("error")
        self._log_print(f"<< {message}")
        messagebox.showerror("Motor Error", message)

    def _on_close(self) -> None:
        try:
            self._disconnect()
        finally:
            self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


def main() -> None:
    DirectMotorGui().run()


if __name__ == "__main__":
    main()
