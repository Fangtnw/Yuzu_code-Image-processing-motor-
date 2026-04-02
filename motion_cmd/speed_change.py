import os
import serial
import time
import re

def create_speed_configured_html(target_speed_mms, output_filename="speed_control.html"):
    """
    1mms.htmlをベースに速度を書き換え、新しいHTMLファイルとして保存する。
    """
    # 1. 速度計算 (1mm/s = 1000um/s)
    spd_base = [0xe8, 0x03, 0x00, 0x00] 
    spd_base_cs = [0x5a, 0x58, 0xa5]
    
    spd_um = int(target_speed_mms * 1000)
    spd_target = [
        (spd_um & 0xFF), ((spd_um >> 8) & 0xFF), 
        ((spd_um >> 16) & 0xFF), ((spd_um >> 24) & 0xFF)
    ]
    
    # チェックサム更新用のマスク計算
    spd_mask = (spd_base[0]^spd_base[1]^spd_base[2]^spd_base[3]) ^ \
               (spd_target[0]^spd_target[1]^spd_target[2]^spd_target[3])
    spd_cs = [cs ^ spd_mask for cs in spd_base_cs]

    try:
        with open("1mms.html", "r", encoding="latin-1") as f:
            spd_html = f.read()
            
        # --- HTML内のデータを置換 ---
        # 速度データ本体の置換
        spd_html = spd_html.replace("02 0c e8 03", f"02 0c {spd_target[0]:02x} {spd_target[1]:02x}")
        spd_html = spd_html.replace("00 00 22 0c e8 03", f"{spd_target[2]:02x} {spd_target[3]:02x} 22 0c e8 03")
        
        # チェックサム部分の置換
        spd_html = spd_html.replace("42 0c e8 03 00 00 00 5a", f"42 0c e8 03 00 00 00 {spd_cs[0]:02x}")
        spd_html = spd_html.replace(
            "e8 03 00 00 00 58 00 00 00 00 00 00 00 00 00 a5", 
            f"e8 03 00 00 00 {spd_cs[1]:02x} 00 00 00 00 00 00 00 00 00 {spd_cs[2]:02x}"
        )
        
        # --- 新しいHTMLファイルとして保存 ---
        with open(output_filename, "w", encoding="latin-1") as f:
            f.write(spd_html)
        
        print(f"✅ 新しい速度設定ファイルを保存しました: {output_filename}")
        return spd_html

    except FileNotFoundError:
        print("❌ Error: '1mms.html' が見つかりません。")
        return None

def parse_html_to_bytes(html_content):
    """HTMLから16進数バイト列を抽出"""
    payloads = []
    current_payload = bytearray()
    for line in html_content.splitlines():
        if "Written data" in line and current_payload: #HTMLの中からここから新しい送信データが始まるぞという目印(Written data)を探し、前のデータがまだ処理中だったらそれをリストに保存して次のデータのために中身を空にする
            payloads.append(current_payload)
            current_payload = bytearray()
        elif 'class="s1"' in line and '<pre>' in line: #実際に16進数の数字が書かれている行を特定する
            match = re.search(r'<pre>(.*?)</pre>', line) #HTMLタグで囲まれたテキスト部分だけを正義表現を使って抜き出す
            if match:
                hex_part = match.group(1)[:48]
                for b in hex_part.split():
                    if len(b) == 2:
                        current_payload.append(int(b, 16))
    if current_payload:
        payloads.append(current_payload)
    return payloads

def main():
    PORT = "/dev/ttyACM0"
    target_spd = float(input("設定したい速度 (mm/s) を入力: "))

    # 1. 指定速度でHTMLを作成
    config_html = create_speed_configured_html(target_spd, "speed_control.html")
    
    if config_html:
        # 2. 作成したHTMLからペイロードを抽出
        speed_cmds = parse_html_to_bytes(config_html)
        
        # 3. 実行コマンド(start_run.html)も読み込む
        all_commands = speed_cmds
        try:
            with open("start_run.html", "r", encoding="latin-1") as f:
                all_commands.extend(parse_html_to_bytes(f.read()))
        except FileNotFoundError:
            print("⚠️ start_run.htmlが見つからないため、登録のみ行います。")

        # 4. シリアル送信
        try:
            with serial.Serial(PORT, 19200, parity=serial.PARITY_EVEN, timeout=0.5) as ser:
                for cmd in all_commands:
                    ser.write(cmd)
                    time.sleep(0.1)
                print(f"\n🚀 モーターへ速度 {target_spd} mm/s の設定・実行コマンドを送信しました。")
        except Exception as e:
            print(f"❌ 通信エラー: {e}")

if __name__ == "__main__":
    main()