def main(args):
    import csv, os

    f = open(args[1])
    filename, ext = os.path.splitext(args[1])
    writer = csv.writer(open(filename + '.csv', 'w'))
    writer.writerow(['theta', 'phi', 'psi', 'altitude', 'vx', 'vy', 'vz', 'accelx', 'accely', 'accelz', 'yaw1', 'yaw2'])
    while True:
        line1 = f.readline()
        if len(line1) == 0: break
        line2 = f.readline()
        line3 = f.readline()
        line4 = f.readline()
        line5 = f.readline()

        tokens = line1.split()[2:]
        theta = float(tokens[1])
        phi = float(tokens[3])
        psi = float(tokens[5])

        tokens = line2.split()[2:]
        altitude = float(tokens[0])

        tokens = line3.split()[2:]
        vx = float(tokens[1])
        vy = float(tokens[3])
        vz = float(tokens[5])

        tokens = line4.split()[2:]
        accelx = float(tokens[0][:-1])
        accely = float(tokens[1][:-1])
        accelz = float(tokens[2][:-1])

        tokens = line5.split()[2:]
        yaw1 = float(tokens[0][:-1])
        yaw2 = float(tokens[1][:-1])

        writer.writerow([theta, phi, psi, altitude, vx, vy, vz, accelx, accely, accelz, yaw1, yaw2])



if __name__ == '__main__':
    import sys
    main(sys.argv)

