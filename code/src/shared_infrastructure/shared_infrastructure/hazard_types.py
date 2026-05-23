from enum import Enum

class HazardType(Enum):
    DUST_PLUME = "Dust Plume"
    FIBROUS_HAZARD = "Fibrous Hazard"
    TOXIC_GAS = "Toxic Gas"
    MATERIAL_HANDLING = "Material Handling"
    NOT_SIGNIFICANT = "Not Significant"
    NONE = "None"