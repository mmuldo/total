import typer
import sensor
import serialio
import csv

from pathlib import Path
from typing import Any

app = typer.Typer(help=f'CLI for running water sensor measurements.')

def write_data(
    output_file: str,
    data: dict[str, Any]
):
    '''
    writes data to specified in csv format; if file already exists, will append data

    Paramters
    ---------
    output_file : str
        path to file to write data
    data : dict[str, Any]
        keys are the row headers and values are the data
    '''
    path = Path(output_file)
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w') as f:
            csv_file = csv.writer(f)
            csv_file.writerow(data.keys())

    with open(output_file, 'a') as f:
        csv_file = csv.writer(f)
        csv_file.writerow(data.values())

@app.command()
def envirostats(
    output_file: str = typer.Option('', help='csv file to save results')
):
    '''Gets temperature and pressure measurments.'''
    ser = serialio.init_serial()

    temperature, pressure = sensor.get_temperature_and_pressure(ser)

    if output_file:
        write_data(
            output_file,
            data={
                'Temperature (C)': temperature,
                'Pressure (mBars)': pressure,
            }
        )

    print(f'Temperature: {temperature} C')
    print(f'Pressure: {pressure} mBars')

@app.command()
def single():
    pass

if __name__ == '__main__':
    app()