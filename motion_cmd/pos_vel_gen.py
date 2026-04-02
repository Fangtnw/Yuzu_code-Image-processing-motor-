import os

def generate_combined_command(target_position_mm, target_speed_mms):
    print(f"--- Generating Combined Command File ---")
    print(f"Target Position: {target_position_mm} mm")
    print(f"Target Speed:    {target_speed_mms} mm/s\n")

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
        return

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
        return

    # ==========================================
    # 3. COMBINE FILES AND SAVE
    # ==========================================
    spd_table_content = spd_html.split("<table>")[1].split("</table>")[0]
    
    # --- STRIP THE 06/86 HANDSHAKE FROM SPEED COMMAND ---
    # Find the second instance of "Written data" which marks the start of the actual payload
    marker = "Written data (COM3)"
    first_idx = spd_table_content.find(marker)
    second_idx = spd_table_content.find(marker, first_idx + 1)
    
    if second_idx != -1:
        # Find the start of the <tr> tag for that second payload
        tr_start = spd_table_content.rfind("<tr", 0, second_idx)
        # Slice the table content to only keep everything from that point forward
        spd_table_content = spd_table_content[tr_start:]
    # ----------------------------------------------------

    divider = '\n<tr class="s3"><td colspan="2"><br/><b>--- SPEED COMMAND START ---</b><br/></td></tr>\n'
    combined_html = pos_html.replace("</table>", divider + spd_table_content + "\n</table>")

    with open("custom_motion.html", "w", encoding="latin-1") as f:
        f.write(combined_html)
        
    print("✅ Success: 'custom_motion.html' generated successfully without duplicate handshakes!")

# ==========================================
# Run the generator!
# ==========================================
generate_combined_command(target_position_mm=15.0, target_speed_mms=4.0)