from matplotlib import pyplot as plt
import pandas as pd


def main():
    df = pd.read_csv('sensor_data.csv')
    fig1, ax1 = plt.subplots(tight_layout=True)
    ax1.plot(df['time'], df['ahrs_pitch'])
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Pitch angle')
    ax1.set_title(r'Pitch Estimation', fontsize=16)
    fig1.savefig('ExamplePlots/Pitch_estimation')

    fig2, ax2 = plt.subplots(tight_layout=True)
    ax2.plot(df['time'], df['ahrs_roll'])
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Roll angle')
    ax2.set_title(r'Roll Estimation', fontsize=16)
    fig2.savefig('ExamplePlots/Roll_estimation')

    fig3, ax3 = plt.subplots(tight_layout=True)
    ax3.plot(df['time'], df['gyX'], df['time'],
             df['gyY'], df['time'], df['gyZ'])
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Rotations')
    ax3.set_title(r'Rate of rotation', fontsize=16)
    fig3.savefig('ExamplePlots/rate_of_rotation')

    fig4, ax4 = plt.subplots(tight_layout=True)
    ax4.plot(df['time'], df['accX'], df['time'],
             df['accY'], df['time'], df['accZ'])
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Accelerations')
    ax4.set_title(r'Acceleration', fontsize=16)
    fig4.savefig('acceleration_plot')
    fig4.savefig('ExamplePlots/acceleration')


if __name__ == '__main__':
    main()
