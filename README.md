# Uncertainty-Control
Semester Thesis

## Introduction
- Case 0, baseline
- Case 1, require no sensor, only based on the estimation of accleration of baseline case
- Case 2, require weight sensor, g sensor used as default
- Case 3, require angle and weight sensors, g sensor used as default
- Case 4, require angle, weight and friction sensors, g sensor used as default
- Case 5, have its own control goal based on reference parameter
- Case 6, case 2 + case 3 == Feedforward + Feedback

## How to use the project?
- If one want to change the control type after used some cases, just simply set `u=0` 
and rerun the `model.slx` and `DataAnalysis.m` script, after that rechoose the control type 
in `DataEstimationAndConstraints.m`.

- Case 1, 2, 3, 4
    - DataSampling.m                    -> config parameters 
    - model.slx                         -> run the model
    - DataAnalysis.m                    -> refine and visulize data 
    - DataEstimationAndConstraints.m    -> derive control input
    - model.slx                         -> rerun the model
    - DataAnalysis.m                    -> refine and visulize data 

- Case 5 (feedback form control)
    - DataSampling.m                    -> config parameters 
    - DataEstimationAndConstraints.m    -> set reference parameter
    - FBmodel.slx                       -> run the model
    - FBDataAnalysis.m                  -> refine and visulize data 

- Case 6 (synthesed feedforward and feedback form control)
    - DataSampling.m                    -> config parameters 
    - model.slx                         -> run the model to get neccessary acceleration estiomation which used in feedforward control
    - DataAnalysis.m                    -> refine and visulize data 
    - DataEstimationAndConstraints.m    -> choose one of cases (1-4) as feedforward control
    - model.slx                         -> rerun model to get the reference velocity
    - DataEstimationAndConstraints.m    -> set the case to "6"
    - FFFBmodel.slx                     -> run model
    - FBDataAnalysis.m                  -> refine and visulize data 
