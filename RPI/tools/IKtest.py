import math

# --- Leg geometry ---
FEMUR = 60
TIBIA = 110

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def calculate_ik(x, y, z):        
        L = math.sqrt((x**2 + y**2) + z**2)

        # J3
        J3 = (FEMUR**2 + TIBIA**2 - L**2) / (2 * FEMUR * TIBIA)
        J3 = math.degrees(math.acos(J3))

        # B
        B = (L**2 + FEMUR**2 - TIBIA**2) / (2 * L * FEMUR)
        B = math.degrees(math.acos(B))

        # A
        A = math.degrees(math.atan2(-z, math.sqrt(x**2+y**2)))

        J2 = B - A

        #J1
        J1 = math.degrees(math.atan(x/y))

        S1Angle = 90 + J1 
        S2Angle = 90 - J2
        S3Angle = J3

        print(f"x={x} y={y} z={z} L={L:.2f} J3={J3:.2f} B={B:.2f} A={A:.2f} J2={J2:.2f}")

        return S1Angle, S2Angle, S3Angle


# --- Test loop ---
if __name__ == "__main__":
    print("IK test. Enter y and z. Ctrl+C to exit.\n")

    while True:
        try:
            x = float(input("Enter x: "))
            y = float(input("Enter y: "))
            z = float(input("Enter z: "))

            S1, S2, S3 = calculate_ik(x, y, z)

            print(f"\nComputed Angles:")
            print(f"  S1: {S1:.2f}°")
            print(f"  S2: {S2:.2f}°")
            print(f"  S3: {S3:.2f}°")
            print("-" * 30)

        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except Exception as e:
            print(f"Error: {e}\n")