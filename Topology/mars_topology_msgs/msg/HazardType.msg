# Emission of gas due to pressure or to chemical reaction
uint8 HAZARD_IDENTIFICATION_NUMBER_2 = 2
# Flammability of liquids (vapours) and gases or self-heating liquid
uint8 HAZARD_IDENTIFICATION_NUMBER_3 = 3
# Flammability of solids or self-heating solid
uint8 HAZARD_IDENTIFICATION_NUMBER_4 = 4
# Oxidizing (fire-intensifying) effect
uint8 HAZARD_IDENTIFICATION_NUMBER_5 = 5
# Toxicity or risk of infection
uint8 HAZARD_IDENTIFICATION_NUMBER_6 = 6
# Radioactivity
uint8 HAZARD_IDENTIFICATION_NUMBER_7 = 7
# Corrosivity
uint8 HAZARD_IDENTIFICATION_NUMBER_8 = 8
# Risk of spontaneous violent reaction
uint8 HAZARD_IDENTIFICATION_NUMBER_9 = 9
# Without special danger (only as placeholder for the second digit)
uint8 HAZARD_IDENTIFICATION_NUMBER_0 = 0

# Reacts dangerously with water (X)
bool react_with_water
# digit one
uint8 main_hazard
# digit two
uint8 additional_hazard_1
# digit three
uint8 additional_hazard_2