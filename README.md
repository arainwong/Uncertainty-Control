# Uncertainty-Control
Semester Thesis

## How to use the project?
- Case 1, 2
    - DataSampling.m                    -> config parameters 
    - model.slx                         -> run model
    - DataAnalysis.m                    -> refine and visulize data 
    - DataEstimationAndConstraints.m    -> derive control input
    - model.slx                         -> rerun model
    - DataAnalysis.m                    -> refine and visulize data 
- Case 3 (feedback form control)
    - DataSampling.m                    -> config parameters 
    - FBmodel.slx                       -> run model
    - DataAnalysis.m                    -> refine and visulize data 
    - DataEstimationAndConstraints.m    -> set reference parameter
    - model.slx                         -> rerun model
    - DataAnalysis.m                    -> refine and visulize data 