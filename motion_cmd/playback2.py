import os
import re
import time
import serial

def create_combined_html(target_position_mm, target_speed_mms):
    print(f"\n--- Calculating Hex Payloads ---")
    
    # ==========================================
    # 1. POSITION CALCULATION (Baseline: 25mm)
    # ==========================================
    pos_base = [0xa8, 0x61, 0x00, 0x00]
    pos_base_cs = [0xb9, 0xf9, 0x85]
    
    pos_um = int(target_position_mm * 1000)
    pos_target = [
        (pos_um & 0xFF), ((pos_um >> 8) & 0xFF), 
        ((pos_um >> 16) & 0xFF), ((pos_um >> 24) & 0xFF)
    ]
    
    pos_mask = (pos_base[0]^pos_base[1]^pos_base[2]^pos_base[3]) ^ \
               (pos_target[0]^pos_target[1]^pos_target[2]^pos_target[3])
    pos_cs = [cs ^ pos_mask for cs in pos_base_cs]

    try:
        with open("write25mm.html", "r", encoding="latin-1") as f:
            pos_html = f.read()
            
        pos_html = pos_html.replace(
            "01 0c a8 61 00 00", 
            f"01 0c {pos_target[0]:02x} {pos_target[1]:02x} {pos_target[2]:02x} {pos_target[3]:02x}"
        )
        pos_html = pos_html.replace("81 0c 00 00 00 00 00 b9", f"81 0c 00 00 00 00 00 {pos_cs[0]:02x}")
        pos_html = pos_html.replace(
            "01 0f 00 00 00 00 00 f9 00 00 00 00 00 00 00 85", 
            f"01 0f 00 00 00 00 00 {pos_cs[1]:02x} 00 00 00 00 00 00 00 {pos_cs[2]:02x}"
        )
    except FileNotFoundError:
        print("❌ Error: Could not find 'write25mm.html'.")
        return None

    # ==========================================
    # 2. SPEED CALCULATION (Baseline: 1 mm/s)
    # ==========================================
    spd_base = [0xe8, 0x03, 0x00, 0x00]
    spd_base_cs = [0x5a, 0x58, 0xa5]
    
    spd_um = int(target_speed_mms * 1000)
    spd_target = [
        (spd_um & 0xFF), ((spd_um >> 8) & 0xFF), 
        ((spd_um >> 16) & 0xFF), ((spd_um >> 24) & 0xFF)
    ]
    
    spd_mask = (spd_base[0]^spd_base[1]^spd_base[2]^spd_base[3]) ^ \
               (spd_target[0]^spd_target[1]^spd_target[2]^spd_target[3])
    spd_cs = [cs ^ spd_mask for cs in spd_base_cs]

    try:
        with open("1mms.html", "r", encoding="latin-1") as f:
            spd_html = f.read()
            
        spd_html = spd_html.replace("02 0c e8 03", f"02 0c {spd_target[0]:02x} {spd_target[1]:02x}")
        spd_html = spd_html.replace("00 00 22 0c e8 03", f"{spd_target[2]:02x} {spd_target[3]:02x} 22 0c e8 03")
        spd_html = spd_html.replace("42 0c e8 03 00 00 00 5a", f"42 0c e8 03 00 00 00 {spd_cs[0]:02x}")
        spd_html = spd_html.replace(
            "e8 03 00 00 00 58 00 00 00 00 00 00 00 00 00 a5", 
            f"e8 03 00 00 00 {spd_cs[1]:02x} 00 00 00 00 00 00 00 00 00 {spd_cs[2]:02x}"
        )
    except FileNotFoundError:
        print("❌ Error: Could not find '1mms.html'.")
        return None

    # ==========================================
    # 3. COMBINE FILES
    # ==========================================
    spd_table_content = spd_html.split("<table>")[1].split("</table>")[0]
    
    marker = "Written data (COM3)"
    first_idx = spd_table_content.find(marker)
    second_idx = spd_table_content.find(marker, first_idx + 1)
    
    if second_idx != -1:
        tr_start = spd_table_content.rfind("<tr", 0, second_idx)
        spd_table_content = spd_table_content[tr_start:]

    divider = '\n<tr class="s3"><td colspan="2"><br/><b>--- SPEED COMMAND START ---</b><br/></td></tr>\n'
    combined_html = pos_html.replace("</table>", divider + spd_table_content + "\n</table>")

    return combined_html

def parse_html_content(html_content):
    """Parses MEXE02 HTML text and extracts all written hex payloads into bytearrays."""
    payloads = []
    current_payload = bytearray()
    lines = html_content.splitlines()
    
    for line in lines:
        if "Written data" in line:
            if current_payload:
                payloads.append(current_payload)
                current_payload = bytearray()
                
        elif 'class="s1"' in line and '<pre>' in line:
            match = re.search(r'<pre>(.*?)</pre>', line)
            if match:
                pre_text = match.group(1)
                hex_part = pre_text[:48]
                for b in hex_part.split():
                    if len(b) == 2:
                        try:
                            current_payload.append(int(b, 16))
                        except ValueError:
                            pass
                            
    if current_payload:
        payloads.append(current_payload)
        
    return payloads

def execute_on_motor_with_heartbeat(port, payloads):
    """Sends payloads to the COM port, then drops into an infinite watchdog ping loop."""
    try:
        with serial.Serial(
            port=port,
            baudrate=19200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
            write_timeout=0.5
        ) as ser:
            
            print(f"\n✅ Opened {port} successfully. Starting sequence of {len(payloads)} frames...\n")
            
            ping_command = None
            for p in payloads:
                if len(p) > 1 and p[1] == 0x03:
                    ping_command = p
                    break
            
            if not ping_command and len(payloads) > 0:
                ping_command = payloads[0]

            for i, payload in enumerate(payloads):
                print(f"[Frame {i+1}/{len(payloads)}] Writing: {payload.hex(' ')}")
                ser.write(payload)
                time.sleep(0.1) 
                
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"         Response: {response.hex(' ')}")
                print("-" * 60)
            
            print("\n🎉 All action commands sent!")
            print("⏳ Starting INFINITE heartbeat to keep motor alive.")
            print("🔴 Press 'Ctrl+C' to stop the motor and safely close the port...")
            
            ping_count = 0
            try:
                while True:
                    if ping_command:
                        ser.write(ping_command)
                        ping_count += 1
                        
                    time.sleep(0.2) 
                    
                    if ser.in_waiting > 0:
                        ser.read(ser.in_waiting) 
                        
                    if ping_count % 5 == 0:
                        print(f"   Motor alive... Sent {ping_count} pings so far.", end='\r')
                        
            except KeyboardInterrupt:
                print(f"\n\n🛑 Heartbeat manually stopped by user. Sent {ping_count} total keep-alive pings.")
                print("Closing COM port safely.")
                
    except serial.SerialTimeoutException:
        print("\n❌ Error: Write timeout occurred! The device might not be ready to receive data.")
    except serial.SerialException as e:
        print(f"\n❌ Failed to open {port}. Error: {e}")
        print("Are you sure you have permission? (Try: sudo chmod 666 /dev/ttyACM0)")

def main():
    print("=== Oriental Motor Auto-Controller ===")
    com_port = "/dev/ttyACM0" 
    all_payloads = []
    
    # 1. Ask User for Input with Validation Loop
    while True:
        user_input = input("Enter target position (mm) [Max: 29] OR type 'home': ").strip().lower()

        if user_input == "home":
            break  # Exit the loop and proceed to homing

        try:
            user_pos = float(user_input)
            
            # --- NEW LIMIT CHECK HERE ---
            if user_pos > 29:
                print("❌ Error: Position cannot exceed 29. Please enter a valid position.")
                continue  # Restarts the while loop
                
            break  # If we make it here, it's a valid number <= 29, exit loop
            
        except ValueError:
            print("❌ Invalid input. Please enter numerical values or the word 'home'.")

    # ==========================================
    # BRANCH A: HOMING MODE
    # ==========================================
    if user_input == "home":
        print("\n🏠 Homing Mode Activated...")
        try:
            with open("2home.html", "r", encoding="latin-1", errors="ignore") as f:
                home_payloads = parse_html_content(f.read())
                all_payloads.extend(home_payloads)
                print("✅ Successfully parsed '2home.html'")
        except FileNotFoundError:
            print("❌ Error: Could not find '2home.html'. Please ensure it's in the same folder.")
            return

    # ==========================================
    # BRANCH B: CUSTOM MOTION MODE
    # ==========================================
    else:
        # Ask for speed with its own validation loop
        while True:
            try:
                user_spd = float(input("Enter target speed (mm/s): "))
                break
            except ValueError:
                print("❌ Invalid input. Please enter numerical values.")

        # Generate dynamic configuration
        config_html_string = create_combined_html(user_pos, user_spd)
        if not config_html_string:
            return 
            
        config_payloads = parse_html_content(config_html_string)
        all_payloads.extend(config_payloads)

        # Append Start Command
        try:
            with open("start_run.html", "r", encoding="latin-1", errors="ignore") as f:
                start_payloads = parse_html_content(f.read())
                all_payloads.extend(start_payloads)
                print("✅ Successfully parsed 'start_run.html'")
        except FileNotFoundError:
            print("⚠️ Warning: Could not find 'start_run.html'. The motor will be configured but will NOT execute the motion.")

    # 4. Execute everything over Serial!
    if all_payloads:
        execute_on_motor_with_heartbeat(com_port, all_payloads)
    else:
        print("❌ No commands to send. Exiting.")

if __name__ == "__main__":
    main()
