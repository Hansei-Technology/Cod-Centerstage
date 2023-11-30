package org.firstinspires.ftc.teamcode.teamcode.controllers;

public class UniversalStates {
    public int currentRow = 0;

    public int extensionPos = 0;

    public double voltage = 14;

    public enum State
    {
        ARM_DOWN,
        ARM_UP,
        MOVING,
        READY_TO_HANG
    }

    public State state = State.MOVING, previousState = State.MOVING;
}
