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
    conversion_factor: float = typer.Option(None, help='if set, converts all admittance quantities from S to uS/cm using this factor')
):
    '''Gets admittance measurement at a single frequency.'''
    admittance_units = 'S' if conversion_factor is None else 'uS/cm'
    
    ser = serialio.init_serial()

    Y = sensor.get_admittance_single_frequency(frequency, amplitude, ser, show_plot, plot_file, conversion_factor)
    print(f'Admittance: {sensor.format_admittance(Y)} {admittance_units}')

    data = {
        'Frequency (Hz)': frequency,
        f'Conductance ({admittance_units})': Y.real,
        f'Susceptance ({admittance_units})': Y.imag,
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
    magnitude_file: str = typer.Option('', help='csv file to save admittance magnitudes'),
    phase_file: str = typer.Option('', help='csv file to save admittance phases'),
    include_envirostats: bool = typer.Option(True, help='also get temperature and pressure measurements'),
    envirostats_file: str = typer.Option('', help='csv file to save temperature and pressure data'),
    conversion_factor: float = typer.Option(None, help='if set, converts all admittance quantities from S to uS/cm using this factor')
):
    '''Gets admittance measurement over a range of frequencies.'''
    admittance_units = 'S' if conversion_factor is None else 'uS/cm'

    ser = serialio.init_serial()

    magnitudes, phases = sensor.get_admittance_spectrum(amplitude, ser, show_plots, bode_plot_file, conversion_factor)
    print(pd.DataFrame({
        'Frequency (Hz)': sensor.FREQUENCIES_FOR_SPECTROSCOPY,
        f'Magnitude ({admittance_units})': magnitudes,
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

@app.command()
def sweep_from_files(
    show_plots: bool = typer.Option(False, help='presents bode plot immediately'),
    bode_plot_file: str = typer.Option('', help='png file to save bode plot'),
    magnitude_file: str = typer.Option('', help='csv file to save admittance magnitudes'),
    phase_file: str = typer.Option('', help='csv file to save admittance phases'),
    admittance_units: str = typer.Option('S', help='units to use for admittance data (e.g. S or uS/cm)')
):
    '''Processes existing admittance data over a range of frequencies.'''
    mag_df = pd.read_csv(magnitude_file)
    phase_df = pd.read_csv(phase_file)

    avg_mags = mag_df.mean(axis=0)
    avg_phases = phase_df.mean(axis=0)

    print(pd.DataFrame({
        'Frequency (Hz)': sensor.FREQUENCIES_FOR_SPECTROSCOPY,
        f'Magnitude ({admittance_units})': avg_mags,
        'Phase (Radians)': avg_phases,
    }))

    sensor.bode_plot(avg_mags, avg_phases, show_plots, bode_plot_file, admittance_units)


if __name__ == '__main__':
    app()