def calculate_motor_command(target_mm):
    # Baseline data from our known working 25mm file
    BASE_BYTES = [0xA8, 0x61, 0x00, 0x00]
    BASE_CHECKSUMS = [0xB9, 0xF9, 0x85]
    
    # 1. Calculate Target Bytes in micrometers
    target_um = int(target_mm * 1000)
    
    # Convert to 4 bytes (Little-Endian)
    target_bytes = [
        (target_um & 0xFF),
        ((target_um >> 8) & 0xFF),
        ((target_um >> 16) & 0xFF),
        ((target_um >> 24) & 0xFF)
    ]
    
    # 2. Calculate the Difference Mask (XOR)
    # XOR all baseline bytes together
    base_xor = BASE_BYTES[0] ^ BASE_BYTES[1] ^ BASE_BYTES[2] ^ BASE_BYTES[3]
    
    # XOR all target bytes together
    target_xor = target_bytes[0] ^ target_bytes[1] ^ target_bytes[2] ^ target_bytes[3]
    
    # The master mask is the XOR difference between the base and target
    xor_mask = base_xor ^ target_xor
    
    # 3. Readjust the Checksums
    new_checksums = [cs ^ xor_mask for cs in BASE_CHECKSUMS]
    
    # Formatting the output nicely
    target_hex_str = " ".join([f"{b:02x}" for b in target_bytes])
    cs1_hex = f"{new_checksums[0]:02x}"
    cs2_hex = f"{new_checksums[1]:02x}"
    cs3_hex = f"{new_checksums[2]:02x}"
    
    # Print the results
    print(f"--- Calculation for {target_mm} mm ---")
    print(f"Target Micrometers: {target_um}")
    print(f"Payload Bytes (Little-Endian): {target_hex_str}")
    print(f"XOR Difference Mask: {xor_mask:02x}")
    print(f"Checksum 1: {cs1_hex}")
    print(f"Checksum 2: {cs2_hex}")
    print(f"Checksum 3: {cs3_hex}")
    print("\nReplacements for your 25mm baseline file:")
    print(f"- Change payload 'a8 61 00 00' to '{target_hex_str}'")
    print(f"- Change Checksum 1 'b9' to '{cs1_hex}'")
    print(f"- Change Checksum 2 'f9' to '{cs2_hex}'")
    print(f"- Change Checksum 3 '85' to '{cs3_hex}'\n")

# --- Run your tests here ---
# You can change these numbers to whatever distance you need!

calculate_motor_command(17)
