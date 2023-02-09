import numpy as np
import jinja2 as j2
import sys

AMPLITUDE = 1/3.3
OFFSET = 1

OUT_FILE_NAME = 'sine.h'
TEMPLATES_DIR = 'templates'

def sine_table(length: int, max_value: int) -> list[float]:
    sine = np.array([
        round((max_value/2) * (AMPLITUDE*np.sin(2 * np.pi * i / length) + OFFSET))
        for i in range(length)
    ])

    return list(sine)

def generate_file(length: int, max_value: int):
    sine = sine_table(length, max_value)

    env = j2.Environment(
        loader=j2.FileSystemLoader(TEMPLATES_DIR),
        trim_blocks=True,
        lstrip_blocks=True,
    )
    template = env.get_template(f'{OUT_FILE_NAME}.j2')

    output = template.render(length=length, table=sine)

    with open(OUT_FILE_NAME, 'w') as out_file:
        out_file.write(output)

generate_file(int(sys.argv[1]), int(sys.argv[2]))
