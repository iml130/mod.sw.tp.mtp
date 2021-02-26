# Definition of possible results
# In general:
# - a result less zero represents an error
# - a result equal to zero represents
# - a result greater zero represents

# < 0: Error with further information
int8 RESULT_ERROR=-1

# == 0: Success
int8 RESULT_SUCCESS=0

# > 0: Success with additional information
int8 RESULT_SUCCESS_ADDITIONAL_INFO=1

int8 result
string description
