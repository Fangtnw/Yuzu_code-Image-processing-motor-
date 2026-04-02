# 指定された速度で，0mmから10mmまで移動し，3秒間停止．
# その後，25mmまで移動し，1秒間停止し，ホーム(原点)に移動．

import os
import re
import time
import serial

def create_combined_html(target_position_mm): # 目標位置を受け取り，目標位置と指定された速度をまとめたHTMLを作る
    print(f"\n--- Calculating Hex Payloads ---")
    
    # ==========================================
    # 1. POSITION CALCULATION (Baseline: 25mm)
    # ==========================================
    pos_base = [0xa8, 0x61, 0x00, 0x00] # 基準位置
    pos_base_cs = [0xb9, 0xf9, 0x85] # チェックサム
    
    pos_um = int(target_position_mm * 1000) # ミリをマイクロに変換
    pos_target = [
        (pos_um & 0xFF), ((pos_um >> 8) & 0xFF), 
        ((pos_um >> 16) & 0xFF), ((pos_um >> 24) & 0xFF)
    ] # リトルエンディアン(4バイトの塊を下位1バイトから順に取り出す)
    
    # ^(XOR演算)
    # 基準位置と目標位置とのXORをした後，それと基準チェックサムとのXORをする．
    # 仕組み
    # チェックサムは全てのデータをXORしたもの．例:cs_old = A^B^C^D
    # Cが位置データとし，CをC_newに変更したいとする．
    # diff = C^C_new
    # cs_old^diff = (A^B^C^D) ^ (C^C_new)
    # ここで，XORは可換法則，結合法則が成り立つため，A^B^C_new^D = cs_new
    pos_mask = (pos_base[0]^pos_base[1]^pos_base[2]^pos_base[3]) ^ \
               (pos_target[0]^pos_target[1]^pos_target[2]^pos_target[3])
    pos_cs = [cs ^ pos_mask for cs in pos_base_cs]

    # テンプレを書き換え
    try:
        with open("write25mm.html", "r", encoding="latin-1") as f: # 0x00~0xFFまで全てに文字を割り当てているためエラーが起きない
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
    except FileNotFoundError: # ファイルが見つからなかった
        print("❌ Error: Could not find 'write25mm.html'.")
        return None
    
    # ==========================================
    # 2. SPEED READING
    # ==========================================
    with open("speed_control.html", "r", encoding="latin-1") as f:
        spd_html = f.read()

    # ==========================================
    # 3. COMBINE FILES
    # ==========================================
    spd_table_content = spd_html.split("<table>")[1].split("</table>")[0] # HTMLの中の<table>以降を取り出した後，</table>以前を取り出す．つまり，<table>タグの中身を抽出
    
    marker = "Written data (COM3)"
    first_idx = spd_table_content.find(marker)
    second_idx = spd_table_content.find(marker, first_idx + 1) # 2回目に出てくるmarkerを探す．find()は見つからなければ-1を返す
    
    if second_idx != -1: # 無事見つかったら
        tr_start = spd_table_content.rfind("<tr", 0, second_idx) # 0からsecond_idxまでの範囲で，右(末尾側)から"<tr"を探す．
        spd_table_content = spd_table_content[tr_start:] # その<tr>以降だけを抽出

    divider = '\n<tr class="s3"><td colspan="2"><br/><b>--- SPEED COMMAND START ---</b><br/></td></tr>\n'
    combined_html = pos_html.replace("</table>", divider + spd_table_content + "\n</table>") # 位置のテーブルの次に，速度のテーブルを追加して，1つのテーブルを作成

    return combined_html

def parse_html_content(html_content): # HTMLの中から16進数データを取り出して，bytearrayに変換
    """Parses MEXE02 HTML text and extracts all written hex payloads into bytearrays."""
    payloads = []
    current_payload = bytearray() # 空のバイト(0~255の整数)を並べた入れ物を用意
    lines = html_content.splitlines()
    
    for line in lines: # HTMLを1行ずつ上から読む
        if "Written data" in line:
            if current_payload:
                payloads.append(current_payload)
                current_payload = bytearray()
                
        elif 'class="s1"' in line and '<pre>' in line: 
            match = re.search(r'<pre>(.*?)</pre>', line) # 16進数が書かれている行だけ抽出．.(任意の1文字)，*(0回以上)，?(できるだけ短く)
            if match:
                pre_text = match.group(1) #preタグの中身を抽出
                hex_part = pre_text[:48] # 先頭の48文字を抽出
                for b in hex_part.split(): # スペース区切りで16進数を取り出す
                    if len(b) == 2: # 2文字(1バイト)なら
                        try:
                            current_payload.append(int(b, 16)) # 16進数を10進数の数値に変換
                        except ValueError:
                            pass
                            
    if current_payload:
        payloads.append(current_payload)
        
    return payloads

def execute_on_motor_with_heartbeat(port, payloads): # モーターにコマンドを送信して，その後ずっとハートビートを送り続ける
    # ハートビート：「まだ通信が生きている」と相手に知らせ続ける信号
    # 一定時間通信が来なければ以上と判断する仕組み
    """Sends payloads to the COM port, then drops into an infinite watchdog ping loop."""
    try:
        with serial.Serial(
            port=port,
            baudrate=19200, # 通信速度
            bytesize=serial.EIGHTBITS, # 8ビット
            parity=serial.PARITY_EVEN, # 偶数パリティ
            stopbits=serial.STOPBITS_ONE, # ストップビット1
            timeout=0.5,
            write_timeout=0.5
        ) as ser:
            
            print(f"\n✅ Opened {port} successfully. Starting sequence of {len(payloads)} frames...\n")
            
            ping_command = None
            for p in payloads:
                if len(p) > 1 and p[1] == 0x03: # ２バイト目が0x03のフレームを探す(0x03は読み取り系コマンドだからモーターを動かさずに安全に送れる)
                    ping_command = p
                    break
            
            if not ping_command and len(payloads) > 0:
                ping_command = payloads[0] # とりあえず最初のフレームを使う

            for i, payload in enumerate(payloads):
                print(f"[Frame {i+1}/{len(payloads)}] Writing: {payload.hex(' ')}")
                ser.write(payload) #バイト列をモーターに送信
                time.sleep(0.1) 
                
                if ser.in_waiting > 0: # 受信バッファにデータがあるなら
                    response = ser.read(ser.in_waiting) # データを読む(バッファに溜まっていくのを防ぐ)
                    print(f"         Response: {response.hex(' ')}")
                print("-" * 60) # 60個の"-"
            
            print("\n🎉 All action commands sent!")
            print("⏳ Starting INFINITE heartbeat to keep motor alive.")
            print("🔴 Press 'Ctrl+C' to stop the motor and safely close the port...")
            
            # ハートビート
            start_time = time.time()
            ping_count = 0
            duration = 5

            while time.time() - start_time < duration:

                if ping_command:
                    ser.write(ping_command)
                    ping_count += 1

                time.sleep(0.2)

                if ser.in_waiting > 0:
                    ser.read(ser.in_waiting)

                if ping_count != 0 and ping_count % 5 == 0:
                    print(f"Motor alive... Sent {ping_count} pings", end="\r")

            print("\n⏹ Heartbeat finished.")

    except serial.SerialTimeoutException: # 書き込みタイムアウトが起きた(モーターが応答しない，ケーブルが抜けている，など)
        print("\n❌ Error: Write timeout occurred! The device might not be ready to receive data.")
    except serial.SerialException as e: # ポートが開けなかった(ポート名が間違っている，権限がない，など)
        print(f"\n❌ Failed to open {port}. Error: {e}")
        print("Are you sure you have permission? (Try: sudo chmod 666 /dev/ttyACM0)") # 666 = 所有者，グループ，その他，全て読み書き可能

def main():
    print("=== Oriental Motor Auto-Controller ===")
    com_port = "/dev/ttyACM0" # ポート指定

    # ==========================================
    # HOMING MODE
    # ==========================================
    print("\n🏠 Homing Mode Activated...")
    all_payloads = []
    try:
        with open("2home.html", "r", encoding="latin-1", errors="ignore") as f:
            home_payloads = parse_html_content(f.read())
            all_payloads.extend(home_payloads)
            print("✅ Successfully parsed '2home.html'")
    except FileNotFoundError: # ファイルが見つからなかっら
        print("❌ Error: Could not find '2home.html'. Please ensure it's in the same folder.")
        return
    
    # Execute everything over Serial!
    if all_payloads:
        execute_on_motor_with_heartbeat(com_port, all_payloads) # ハートビート
    else:
        print("❌ No commands to send. Exiting.")

    # ==========================================
    # 10mm MOVING MODE
    # ==========================================
    print("\n 10mm MOVING Activated...")
    all_payloads = []
    pos = 10
    config_html_string = create_combined_html(pos) # 位置と速度を埋め込んだHTMLを作る
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
    except FileNotFoundError: # ファイルが見つからなかった
        print("⚠️ Warning: Could not find 'start_run.html'. The motor will be configured but will NOT execute the motion.")

    # Execute everything over Serial!
    if all_payloads:
        execute_on_motor_with_heartbeat(com_port, all_payloads) # ハートビート
    else:
        print("❌ No commands to send. Exiting.")

    # ==========================================
    # 3s WAITING MODE
    # ==========================================
    time.sleep(3)
    print("3swaiting")

    # ==========================================
    # 25mm MOVING MODE
    # ==========================================
    print("\n 25mm MOVING Activated...")
    all_payloads = []
    pos = 25
    config_html_string = create_combined_html(pos) # 位置と速度を埋め込んだHTMLを作る
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
    except FileNotFoundError: # ファイルが見つからなかった
        print("⚠️ Warning: Could not find 'start_run.html'. The motor will be configured but will NOT execute the motion.")

    # Execute everything over Serial!
    if all_payloads:
        execute_on_motor_with_heartbeat(com_port, all_payloads) # ハートビート
    else:
        print("❌ No commands to send. Exiting.")

    # ==========================================
    # 1s WAITING MODE
    # ==========================================
    time.sleep(1)

    # ==========================================
    # HOMING MODE MODE
    # ==========================================
    print("\n🏠 Homing Mode Activated...")
    all_payloads = []
    try:
        with open("2home.html", "r", encoding="latin-1", errors="ignore") as f:
            home_payloads = parse_html_content(f.read())
            all_payloads.extend(home_payloads)
            print("✅ Successfully parsed '2home.html'")
    except FileNotFoundError: # ファイルが見つからなかっら
        print("❌ Error: Could not find '2home.html'. Please ensure it's in the same folder.")
        return

    # Execute everything over Serial!
    if all_payloads:
        execute_on_motor_with_heartbeat(com_port, all_payloads) # ハートビート
    else:
        print("❌ No commands to send. Exiting.")


if __name__ == "__main__":
    main()
