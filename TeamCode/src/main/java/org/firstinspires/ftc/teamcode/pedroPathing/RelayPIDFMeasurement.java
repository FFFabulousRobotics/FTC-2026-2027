package org.firstinspires.ftc.teamcode.pedroPathing;

/** 去年继电辨识法：丢弃起步周期，测完整周期的幅度、周期，再估算 PD。 */
final class RelayPIDFMeasurement {
    final double power, band;
    private final int skip, count;
    private final double[] periods, amplitudes;
    private int direction = 1, cycles, samples;
    private double previousError, previousTime = Double.NaN, crossing = Double.NaN;
    private double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
    double period, amplitude, ultimateGain;

    RelayPIDFMeasurement(double power, double band, int skip, int count) {
        this.power = power; this.band = band; this.skip = skip; this.count = count;
        periods = new double[count]; amplitudes = new double[count];
    }

    double update(double time, double error) {
        if (direction > 0 && error < -band) direction = -1;
        else if (direction < 0 && error > band) direction = 1;
        low = Math.min(low, error); high = Math.max(high, error);
        if (Double.isFinite(previousTime) && previousError < 0 && error >= 0) {
            double at = previousTime + (time - previousTime) * -previousError / (error - previousError);
            if (Double.isFinite(crossing) && ++cycles > skip) {
                periods[samples % count] = at - crossing;
                amplitudes[samples % count] = (high - low) / 2;
                samples++;
            }
            crossing = at;
            low = high = error;
        }
        previousTime = time; previousError = error;
        return direction * power;
    }

    int measuredCycles() { return Math.min(samples, count); }

    boolean ready() {
        if (samples < count) return false;
        period = amplitude = 0;
        for (int i = 0; i < count; i++) { period += periods[i]; amplitude += amplitudes[i]; }
        period /= count; amplitude /= count;
        if (period <= 0 || amplitude < band * 1.5) return false;
        double periodVariance = 0, amplitudeVariance = 0;
        for (int i = 0; i < count; i++) {
            periodVariance += Math.pow(periods[i] - period, 2);
            amplitudeVariance += Math.pow(amplitudes[i] - amplitude, 2);
        }
        if (Math.sqrt(periodVariance / count) > period * 0.3
                || Math.sqrt(amplitudeVariance / count) > amplitude * 0.3) return false;
        ultimateGain = 4 * power / (Math.PI * Math.sqrt(amplitude * amplitude - band * band));
        return Double.isFinite(ultimateGain);
    }

    double[] gains(double feedForward) {
        if (!ready()) throw new IllegalStateException("Relay measurement is not ready");
        double p = Math.min(0.8 * ultimateGain, 0.5 * power / band);
        return new double[]{p, 0, p * period / 8, feedForward};
    }
}
