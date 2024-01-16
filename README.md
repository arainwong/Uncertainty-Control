# Uncertainty-Control
Semester Thesis

## Introduction

### >Case I<
- 1 -> Scenario 1, require no sensor
- 2 -> Scenario 2, require weight sensor, g sensor used as default
- 3 -> Scenario 3, require angle and weight sensors, g sensor used as default
- 4 -> Scenario 4, require angle, weight and friction sensors, g sensor used as default

### >Case II<
- 5 -> Case II (FB), Feedback control, have its own control goal based on reference parameter

### >Case III<
- 6 -> Case III (FF + FB), Feedforward + Feedback

### >Case IV< | additional setting, e.g. Case I Scenario 3 Configuration
- 7 -> Case IV, Minimum energy consumption strategy

## How to use the project?
- If one want to change the control type after used some cases, just simply set `u=0` 
and rerun the `model.slx` and `DataAnalysis.m` script, after that rechoose the control type 
in `DataEstimationAndConstraints.m`.

- 1, 2, 3, 4
    - DataSampling.m                    -> config parameters 
    - model.slx                         -> run the model
    - DataAnalysis.m                    -> refine and visulize data 
    - DataEstimationAndConstraints.m    -> derive control input
    - model.slx                         -> rerun the model
    - DataAnalysis.m                    -> refine and visulize data 

- 5 (Feedback form control)
    - DataSampling.m                    -> config parameters 
    - DataEstimationAndConstraints.m    -> set reference parameter
    - FBmodel.slx                       -> run the model
    - FBDataAnalysis.m                  -> refine and visulize data 

- 6 (Synthesed feedforward and feedback form control)
    - DataSampling.m                    -> config parameters 
    - model.slx                         -> run the model to get neccessary acceleration estiomation which used in feedforward control
    - DataAnalysis.m                    -> refine and visulize data 
    - DataEstimationAndConstraints.m    -> choose one of cases (1-4) as feedforward control
    - model.slx                         -> rerun model to get the reference velocity
    - DataEstimationAndConstraints.m    -> set the case to "6"
    - FFFBmodel.slx                     -> run model
    - FBDataAnalysis.m                  -> refine and visulize data 

- 7 (Minimum energy consumption)
    - DataSampling.m                    -> config parameters 
    - model.slx                         -> run the model 
    - DataAnalysis.m                    -> refine and visulize data 
    - DataEstimationAndConstraints.m    -> set to `optimizationType(7) = True` and choose `type7Type(?) = True`, derive control input
    - model.slx                         -> rerun the model
    - DataAnalysis.m                    -> refine and visulize data 
