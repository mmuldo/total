import typer
import sensor
import serialio
import csv
import pandas as pd

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
    print(f'Temperature: {temperature} C')
    print(f'Pressure: {pressure} mBars')

    if output_file:
        write_data(
            output_file,
            data={
                'Temperature (C)': temperature,
                'Pressure (mBars)': pressure,
            }
        )

@app.command()
def single(
    frequency: int = typer.Argument(..., help=f'frequency of sinusoid (between {sensor.MIN_FREQ_HZ} and {sensor.MAX_FREQ_HZ} Hz)'),
    amplitude: float = typer.Option(0.7, help=f'amplitude of sinusoid (between {sensor.MIN_AMPLITUDE_HZ} and {sensor.MAX_AMPLITUDE_HZ} V)'),
    show_plot: bool = typer.Option(False, help='presents waveforms immediately'),
    plot_file: str = typer.Option('', help='png file to save results'),
    output_file: str = typer.Option('', help='csv file to save results'),
    include_envirostats: bool = typer.Option(True, help='also get temperature and pressure measurements'),
):
    '''Gets impedence measurment at a single frequency.'''
    ser = serialio.init_serial()

    Z = sensor.get_impedence_single_frequency(frequency, amplitude, ser, show_plot, plot_file)
    print(f'Impedence: {sensor.format_impedence(Z)} Ohms')

    data = {
        'Frequency (Hz)': frequency,
        'Resistance (Ohms)': Z.real,
        'Reactance (Ohms)': Z.imag,
    }
    if include_envirostats:
        temperature, pressure = sensor.get_temperature_and_pressure(ser)
        data['Temperature (C)'] = temperature
        data['Pressure (mBars)'] = pressure
        print(f'Temperature: {temperature} C')
        print(f'Pressure: {pressure} mBars')


    if output_file:
        write_data(output_file, data)

@app.command()
def sweep(
    amplitude: float = typer.Option(0.7, help=f'amplitude of sinusoid (between {sensor.MIN_AMPLITUDE_HZ} and {sensor.MAX_AMPLITUDE_HZ} V)'),
    show_plots: bool = typer.Option(False, help='presents bode plot immediately'),
    bode_plot_file: str = typer.Option('', help='png file to save bode plot'),
    magnitude_file: str = typer.Option('', help='csv file to save impedence magnitudes'),
    phase_file: str = typer.Option('', help='csv file to save impedence phases'),
    include_envirostats: bool = typer.Option(True, help='also get temperature and pressure measurements'),
    envirostats_file: str = typer.Option('', help='csv file to save temperature and pressure data'),
):
    ser = serialio.init_serial()

    magnitudes, phases = sensor.get_impedence_spectrum(amplitude, ser, show_plots, bode_plot_file)
    print(pd.DataFrame({
        'Frequency (Hz)': sensor.FREQUENCIES_FOR_SPECTROSCOPY,
        'Magnitude (Ohms)': magnitudes,
        'Phase (Radians)': phases,
    }))

    magnitude_data = {
        frequency: magnitude
        for frequency, magnitude in zip(sensor.FREQUENCIES_FOR_SPECTROSCOPY, magnitudes)
    }
    if magnitude_file:
        write_data(magnitude_file, magnitude_data)

    phase_data = {
        frequency: phase
        for frequency, phase in zip(sensor.FREQUENCIES_FOR_SPECTROSCOPY, phases)
    }
    if phase_file:
        write_data(phase_file, phase_data)

    if include_envirostats:
        temperature, pressure = sensor.get_temperature_and_pressure(ser)
        print(f'Temperature: {temperature} C')
        print(f'Pressure: {pressure} mBars')

        if envirostats_file:
            write_data(
                envirostats_file,
                data={
                    'Temperature (C)': temperature,
                    'Pressure (mBars)': pressure,
                }
            )

if __name__ == '__main__':
    app()