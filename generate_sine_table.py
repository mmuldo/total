import numpy as np
import jinja2 as j2
import sys

VDD = 3.3

OUT_FILE_NAME = 'sinetable.h'
TEMPLATES_DIR = 'templates'

def sine_table(length: int, amplitude: float) -> list[float]:
    sine = np.array([
        round(length * ((2*amplitude/VDD)*np.sin(2 * np.pi * i / length) + 1))
        for i in range(length)
    ])

    return list(sine)

def generate_file(length: int, amplitude: float, filename: str=OUT_FILE_NAME):
    sine = sine_table(length, amplitude)

    env = j2.Environment(
        loader=j2.FileSystemLoader(TEMPLATES_DIR),
        trim_blocks=True,
        lstrip_blocks=True,
    )
    template = env.get_template(f'{OUT_FILE_NAME}.j2')

    output = template.render(length=length, table=sine)

    with open(filename, 'w') as out_file:
        out_file.write(output)

def generate_files(lengths: list[int], amplitude: float):
    tables = {
        length: sine_table(length, amplitude)
        for length in lengths
    }

    env = j2.Environment(
        loader=j2.FileSystemLoader(TEMPLATES_DIR),
        trim_blocks=True,
        lstrip_blocks=True,
    )
    template = env.get_template(f'sinetables.h.j2')

    output = template.render(tables=tables)

    with open('sinetables.h', 'w') as out_file:
        out_file.write(output)



#length = int(sys.argv[1])
#amplitude = float(sys.argv[2])
#filename = sys.argv[3] if len(sys.argv) >= 3 else OUT_FILE_NAME

generate_files([(i+1)*100 for i in range(10)], 1)
