import math

# --- Leg geometry ---
FEMUR = 60
TIBIA = 104


def calculate_ik(y, z):
    # Compute distance from hip joint
    L = math.sqrt(y**2 + z**2)

    # J3 (knee)
    J3 = (FEMUR**2 + TIBIA**2 - L**2) / (2 * FEMUR * TIBIA)
    J3 = math.degrees(math.acos(J3))

    # B (shoulder triangle angle)
    B = (L**2 + FEMUR**2 - TIBIA**2) / (2 * L * FEMUR)
    B = math.degrees(math.acos(B))

    # A (foot angle relative to horizontal)
    A = math.degrees(math.atan2(-z , y))

    # J2 (shoulder)
    J2 = B - A

    S2Angle = 90 - J2
    S3Angle = J3
    print(f"y={y} z={z} L={L:.2f} J3={J3:.2f} B={B:.2f} A={A:.2f} J2={J2:.2f} S2={S2Angle:.2f} S3={S3Angle:.2f}")

    return S2Angle, S3Angle


# --- Test loop ---
if __name__ == "__main__":
    print("IK test. Enter y and z. Ctrl+C to exit.\n")

    while True:
        try:
            y = float(input("Enter y: "))
            z = float(input("Enter z: "))

            S2, S3 = calculate_ik(y, z)

            print(f"\nComputed Angles:")
            print(f"  S2 (shoulder): {S2:.2f}°")
            print(f"  S3 (knee):     {S3:.2f}°")
            print("-" * 30)

        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except Exception as e:
            print(f"Error: {e}\n")