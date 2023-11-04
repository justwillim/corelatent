int8 order 0 # order of the polynomial 2 for quadratic, 3 for cubic
int32 dim 1 # dimension of the polynomial 3 for 3D space, 2 for 2D space
int32 piece 1 # piece of the polynomial 1 for the first piece, 2 for the second piece
# polynomial \sum a_i x^i coefficients = [a_00, a_01, ..., b_00, b_01, ..., a_10, a_11, ...]
# i=0,1,...,order
# Redundant information for validation (order+1) * dim * piece = coefficients.size
float64[] coefficients [1]
# time allocation for each piece, t_0 = 0, t_i > t_{i-1}, t_{piece} = T
# only store t_1, t_2, ..., t_{piece}, totally 'piece' points
float64[] time_allocation [1]