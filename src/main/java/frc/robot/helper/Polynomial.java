package frc.robot.helper;
import java.io.*;
import java.util.*;

public class Polynomial implements Serializable {
    private double[] coefficients;
    private int degree;
    public Polynomial(double[] coefficients){
        this.coefficients=coefficients;
        degree = coefficients.length-1;
    }
    public double getOutput(double x){
        double ret = 0;
        for (int i=0;i<coefficients.length;i++){
            ret += coefficients[i]*Math.pow(x,i);
        }
        return ret;
    }
    @Override
    public boolean equals(Object obj) {
        Polynomial other = (Polynomial) obj;
        return (Arrays.equals(coefficients, other.coefficients));
    }
    public String toString(){
        StringBuilder ret = new StringBuilder();
        ret.append(coefficients[0]);
        for (int i=1;i<coefficients.length;i++){
            ret.append(" + "+coefficients[i]+"x^"+i);
        }
        return ret.toString();
    }
}
