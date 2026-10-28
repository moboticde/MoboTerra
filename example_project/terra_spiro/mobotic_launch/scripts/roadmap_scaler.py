import xml.etree.ElementTree as ET
import argparse
import os

def scale_roadmap(input_file, scale_factor):
    try:
        # Parse the XML file
        tree = ET.parse(input_file)
        root = tree.getroot()

        # Scale POI positions
        for poi in root.findall('.//points_of_interest/poi'):
            position = poi.find('position')
            if position is not None:
                x = float(position.get('x')) * scale_factor
                y = float(position.get('y')) * scale_factor
                position.set('x', str(x))
                position.set('y', str(y))

        # Scale Curve control points (cp)
        for curve in root.findall('.//curves/curve'):
            control_points = curve.find('control_points')
            if control_points is not None:
                for cp in control_points.findall('cp'):
                    x = float(cp.get('x')) * scale_factor
                    y = float(cp.get('y')) * scale_factor
                    cp.set('x', str(x))
                    cp.set('y', str(y))

        # Construct the output filename with the scale factor
        base_name, ext = os.path.splitext(input_file)
        # Format the scale factor to avoid overly long floats in the filename if not needed
        # e.g., 2.0 instead of 2.000000
        formatted_scale = f"{scale_factor:g}"
        output_file = f"scaled-{formatted_scale}_{base_name}{ext}"

        # Save the modified XML
        tree.write(output_file, encoding='utf-8', xml_declaration=True)
        print(f"Successfully scaled roadmap by a factor of {scale_factor}.")
        print(f"Output saved to: {output_file}")

    except FileNotFoundError:
        print(f"Error: The file '{input_file}' was not found.")
    except ET.ParseError:
        print(f"Error: The file '{input_file}' is not a valid XML file.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Scale an XML roadmap's coordinates by a given factor.")
    parser.add_argument("input_file", type=str, help="Path to the template XML roadmap file.")
    parser.add_argument("scale_factor", type=float, help="Multiplier for the X and Y coordinates (e.g., 2.0 to double size).")

    args = parser.parse_args()

    # Run the scaling function
    scale_roadmap(args.input_file, args.scale_factor)
