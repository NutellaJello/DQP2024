package org.firstinspires.ftc.teamcode.subsystems.States;

public class StateManager {
    private SampleManualState sampleManualState;
    private SampleSlideDownState sampleSlideDownState;
    private SampleSlideUpState sampleSlideUpState;
    private SampleTransferState sampleTransferState;
    private SpecimenTransferState specimenTransferState;

    public StateManager() {
        sampleManualState = SampleManualState.SAMPLE_MANUAL_INIT;
        sampleSlideDownState = SampleSlideDownState.SAMPLE_SLIDE_DOWN_INIT;
        sampleSlideUpState = SampleSlideUpState.SAMPLE_SLIDE_UP_INIT;
        sampleTransferState = SampleTransferState.SAMPLE_TRANSFER_INIT;
    }

    public SampleManualState getSampleManualState() {
        return sampleManualState;
    }

    public void setSampleManualState(SampleManualState newState) {
        sampleManualState = newState;
    }

    public SampleSlideDownState getSampleSlideDownState() {
        return sampleSlideDownState;
    }

    public void setSampleSlideDownState(SampleSlideDownState newState) {
        sampleSlideDownState = newState;
    }

    public SampleSlideUpState getSampleSlideUpState() {
        return sampleSlideUpState;
    }

    public void setSampleSlideUpState(SampleSlideUpState newState) {
        sampleSlideUpState = newState;
    }

    public SampleTransferState getSampleTransferState() {
        return sampleTransferState;
    }

    public void setSampleTransferState(SampleTransferState newState) {
        sampleTransferState = newState;
    }

    public SpecimenTransferState getSpecimenTransferState() {
        return specimenTransferState;
    }

    public void setSpecimenTransferState(SpecimenTransferState newState) {
        specimenTransferState = newState;
    }
}

