package frc.robot.utils;


public class Smooth {
    private final double[] window;
    private int size, index = 0;
    private double sum = 0.0;


    public Smooth(int size) {
        this.size = size;
        window = new double[size];
    }


    public double calculate(double newValue) {
        sum -= window[index]; // Remove oldest value from sum
        window[index] = newValue; // Add new value to window
        sum += newValue; // Add new value to sum
        index = (index + 1) % size; // Increment index and wrap around
        return sum / size; // Return the average
    }
}