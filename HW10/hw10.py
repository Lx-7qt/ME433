import os
import csv
import numpy as np
import matplotlib.pyplot as plt
#All images are in the plots folder
OUTPUT_DIR = "plots"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def read_csv_signal(filepath):
    t = []
    data = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            try:
                t.append(float(row[0]))
                data.append(float(row[1]))
            except ValueError:
                continue
    return np.array(t), np.array(data)

def compute_sample_rate(t):
    n = len(t)
    total_time = t[-1] - t[0]
    return n / total_time

def compute_fft(signal, fs):
    n = len(signal)
    Y = np.fft.fft(signal) / n
    freqs = np.fft.fftfreq(n, d=1/fs)
    half = n // 2
    return freqs[:half], np.abs(Y[:half])


def plot_signal_and_fft(t, data, title_prefix):
    fs = compute_sample_rate(t)
    freqs, mags = compute_fft(data, fs)

    fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6))
    fig.suptitle(f'{title_prefix}: Raw Signal & FFT', fontsize=14)

    # Time-domain plot
    ax_time.plot(t, data, color='black', linewidth=1)
    ax_time.set_xlabel('Time [s]')
    ax_time.set_ylabel('Amplitude')
    ax_time.set_title('Signal vs Time')

    # Frequency-domain plot (log-log)
    ax_freq.loglog(freqs, mags, color='black', linewidth=1)
    ax_freq.set_xlabel('Frequency [Hz]')
    ax_freq.set_ylabel('|Y(f)|')
    ax_freq.set_title('FFT (One-sided)')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    outpath = os.path.join(OUTPUT_DIR, f"{title_prefix}_raw.png")
    fig.savefig(outpath, dpi=150)
    plt.close(fig)

def moving_average_filter(signal, window_size):
    """
    Applies a moving average filter of length `window_size` to `signal`.
    Pads the beginning so output has the same length as input.
    """
    n = len(signal)
    filtered = np.zeros(n)
    cumsum = np.cumsum(np.insert(signal, 0, 0.0))
    for i in range(n):
        if i < window_size:
            filtered[i] = cumsum[i+1] / (i+1)
        else:
            filtered[i] = (cumsum[i+1] - cumsum[i+1-window_size]) / window_size
    return filtered

def demo_moving_average(csv_files, window_sizes):
    for filepath in csv_files:
        if not os.path.isfile(filepath):
            continue
        label = os.path.splitext(os.path.basename(filepath))[0]
        t, data = read_csv_signal(filepath)
        fs = compute_sample_rate(t)

        # Save raw plot once
        plot_signal_and_fft(t, data, title_prefix=label)

        for w in window_sizes:
            filtered = moving_average_filter(data, w)
            freqs_raw, mags_raw = compute_fft(data, fs)
            freqs_filt, mags_filt = compute_fft(filtered, fs)

            fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6))
            fig.suptitle(f'{label}: MAF (window={w})', fontsize=14)

            # Time-domain comparison
            ax_time.plot(t, data, color='black', label='Unfiltered')
            ax_time.plot(t, filtered, color='red', label='Filtered')
            ax_time.set_xlabel('Time [s]')
            ax_time.set_ylabel('Amplitude')
            ax_time.legend()
            ax_time.set_title('Signal vs Time')

            # Frequency-domain comparison
            ax_freq.loglog(freqs_raw, mags_raw, color='black', label='Unfiltered')
            ax_freq.loglog(freqs_filt, mags_filt, color='red', label='Filtered')
            ax_freq.set_xlabel('Frequency [Hz]')
            ax_freq.set_ylabel('|Y(f)|')
            ax_freq.legend()
            ax_freq.set_title('FFT vs Frequency')

            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            outname = f"{label}_MAF_w{w}.png"
            outpath = os.path.join(OUTPUT_DIR, outname)
            fig.savefig(outpath, dpi=150)
            plt.close(fig)
def exponential_moving_average(signal, A):
    n = len(signal)
    y = np.zeros(n)
    y[0] = signal[0]
    B = 1.0 - A
    for i in range(1, n):
        y[i] = A * y[i-1] + B * signal[i]
    return y

def demo_iir(csv_files, A_values):
    for filepath in csv_files:
        if not os.path.isfile(filepath):
            continue
        label = os.path.splitext(os.path.basename(filepath))[0]
        t, data = read_csv_signal(filepath)
        fs = compute_sample_rate(t)

        # Save raw plot once
        plot_signal_and_fft(t, data, title_prefix=label)

        for A in A_values:
            filtered = exponential_moving_average(data, A)
            freqs_raw, mags_raw = compute_fft(data, fs)
            freqs_filt, mags_filt = compute_fft(filtered, fs)

            fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6))
            fig.suptitle(f'{label}: IIR EMA (A={A:.2f}, B={1-A:.2f})', fontsize=14)

            # Time-domain comparison
            ax_time.plot(t, data, color='black', label='Unfiltered')
            ax_time.plot(t, filtered, color='red', label='Filtered')
            ax_time.set_xlabel('Time [s]')
            ax_time.set_ylabel('Amplitude')
            ax_time.legend()
            ax_time.set_title('Signal vs Time')

            # Frequency-domain comparison
            ax_freq.loglog(freqs_raw, mags_raw, color='black', label='Unfiltered')
            ax_freq.loglog(freqs_filt, mags_filt, color='red', label='Filtered')
            ax_freq.set_xlabel('Frequency [Hz]')
            ax_freq.set_ylabel('|Y(f)|')
            ax_freq.legend()
            ax_freq.set_title('FFT vs Frequency')

            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            outname = f"{label}_IIR_A{A:.2f}.png"
            outpath = os.path.join(OUTPUT_DIR, outname)
            fig.savefig(outpath, dpi=150)
            plt.close(fig)


def design_fir_filter(num_taps, cutoff_hz, fs, window_type='hamming'):

    wc = cutoff_hz / (fs / 2.0)  # normalized cutoff (0 < wc < 1)
    M = (num_taps - 1) / 2.0
    n = np.arange(num_taps)
    hd = wc * np.sinc(wc * (n - M))

    if window_type.lower() == 'hamming':
        win = np.hamming(num_taps)
    elif window_type.lower() == 'hann':
        win = np.hanning(num_taps)
    elif window_type.lower() == 'blackman':
        win = np.blackman(num_taps)

    h = hd * win
    h /= np.sum(h)  # normalize DC gain
    return h

def apply_fir_filter(signal, h):
    return np.convolve(signal, h, mode='same')

def demo_fir(csv_files, fir_configs):
    for filepath in csv_files:
        if not os.path.isfile(filepath):
            continue
        label = os.path.splitext(os.path.basename(filepath))[0]
        t, data = read_csv_signal(filepath)
        fs = compute_sample_rate(t)

        # Save raw plot once
        plot_signal_and_fft(t, data, title_prefix=label)

        for cfg in fir_configs:
            num_taps = cfg['num_taps']
            cutoff_hz = cfg['cutoff_hz']
            window_type = cfg['window']

            h = design_fir_filter(num_taps=num_taps, cutoff_hz=cutoff_hz, fs=fs, window_type=window_type)
            filtered = apply_fir_filter(data, h)
            freqs_raw, mags_raw = compute_fft(data, fs)
            freqs_filt, mags_filt = compute_fft(filtered, fs)

            fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6))
            fig.suptitle(
                f"{label}: FIR (N={num_taps}, fc={cutoff_hz}Hz, window={window_type})",
                fontsize=14
            )

            # Time-domain comparison
            ax_time.plot(t, data, color='black', label='Unfiltered')
            ax_time.plot(t, filtered, color='red', label='Filtered')
            ax_time.set_xlabel('Time [s]')
            ax_time.set_ylabel('Amplitude')
            ax_time.legend()
            ax_time.set_title('Signal vs Time')

            # Frequency-domain comparison
            ax_freq.loglog(freqs_raw, mags_raw, color='black', label='Unfiltered')
            ax_freq.loglog(freqs_filt, mags_filt, color='red', label='Filtered')
            ax_freq.set_xlabel('Frequency [Hz]')
            ax_freq.set_ylabel('|Y(f)|')
            ax_freq.legend()
            ax_freq.set_title('FFT vs Frequency')

            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            outname = f"{label}_FIR_N{num_taps}_fc{cutoff_hz}.png"
            outpath = os.path.join(OUTPUT_DIR, outname)
            fig.savefig(outpath, dpi=150)
            plt.close(fig)
if __name__ == "__main__":
    csv_files = ['sigA.csv', 'sigB.csv', 'sigC.csv', 'sigD.csv']

    # Part 1: Plot raw signals & FFTs
    for filepath in csv_files:
        if os.path.isfile(filepath):
            t, data = read_csv_signal(filepath)
            plot_signal_and_fft(t, data, title_prefix=os.path.splitext(os.path.basename(filepath))[0])

    # Part 2: Moving Average Filter (trying a few window sizes and selecting the best one)
    maf_window_sizes = [20, 100, 500]
    demo_moving_average(csv_files, maf_window_sizes)

    # Part 3: Exponential Moving Average (IIR) Filter (trying a few A values and selecting the best one)
    iir_A_values = [0.9, 0.95, 0.99]
    demo_iir(csv_files, iir_A_values)

    # Part 4: FIR Filter via Sinc + Window (trying a few configs and selecting the best one)
    fir_configs = [
        {'num_taps': 51, 'cutoff_hz': 50, 'window': 'hamming'},
        {'num_taps': 101, 'cutoff_hz': 30, 'window': 'hann'},
        {'num_taps': 101, 'cutoff_hz': 20, 'window': 'blackman'}
    ]
    demo_fir(csv_files, fir_configs)

    print("Done")
