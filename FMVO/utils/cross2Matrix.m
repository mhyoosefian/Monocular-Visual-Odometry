function M = cross2Matrix(x)

% Computes the skewsymmetric matrix corresponding to the vector x

M = [0    -x(3)  x(2);
     x(3)   0   -x(1);
    -x(2)  x(1)   0  ];

