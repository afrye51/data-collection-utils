import csv
import argparse

def csv_to_kml(csv_file, kml_file, skip=0):
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        points = [(row['latitude'], row['longitude']) for row in reader]

    with open(kml_file, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('<Document>\n')
        f.write('<name>Path</name>\n')
        f.write('<Placemark>\n')
        f.write('<LineString>\n')
        f.write('<tessellate>1</tessellate>\n')
        f.write('<coordinates>\n')

        i = 0
        for lat, lon in points:
            if i % (skip+1):
                f.write(f'{lon},{lat},0\n')  # KML format is lon,lat,alt
            i += 1

        f.write('</coordinates>\n')
        f.write('</LineString>\n')
        f.write('</Placemark>\n')
        f.write('</Document>\n')
        f.write('</kml>\n')

    print(f"KML file saved to {kml_file}")

def main(prefix, skip):
    csv_to_kml(f'{prefix}_inspvax.csv', f'{prefix}_inspvax.kml', skip=skip)
    csv_to_kml(f'{prefix}_gps.csv', f'{prefix}_gps.kml', skip=skip)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", help="Prefix for input files, e.g., 'run1' for 'run1_gps.csv' and 'run1_inspvax.csv'")
    parser.add_argument("skip_lines", help="To decrease the output file size (1 -> keep every other line, 3 -> skip three, then take one)")
    args = parser.parse_args()

    main(args.prefix, int(args.skip_lines))
