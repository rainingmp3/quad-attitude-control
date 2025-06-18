import numpy as np
def case(n):
    """Returns inputs to specifid flight case"""
    case_1 = { #1st case
                    "attMass":0,
                    "initPhi":0,
                    "initTheta":0}

    case_2 = { #2nd case
                    "attMass":1,
                    "initPhi":0,
                    "initTheta":0}


    case_3 = { #3rd case
                    "attMass":1,
                    "initPhi":0,
                    "initTheta":3.14/18}


    case_4 = { #4th case
                    "attMass":1.5,
                    "initPhi":3.14/6,
                    "initTheta":3.14/9}

    case_angle = { #4th case
                    "attMass":0,
                    "initPhi":3.14/4,
                    "initTheta":0}
    
    cases = [case_1,case_2,case_3,case_4,case_angle]

    return  cases[n-1]
