int8 order [0] # order of the polynomial 2 for quadratic, 3 for cubic
int32 dim [1] # dimension of the polynomial 3 for 3D space, 2 for 2D space
# polynomial \sum a_i x^i coefficients = [a_0, a_1, ..., b_0, b_1, ...]
# i=0,1,...,order
# Redundant information for validation (order+1) * dim = coefficients.size
float64[] coefficients [1]