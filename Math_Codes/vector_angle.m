function angle = vector_angle( v1, v2 )

angle = acos( dot(v1,v2) / norm(v1) / norm(v2) );

% imaginary error handling
if abs( imag(angle) ) < 1e-6
    angle = real(angle);
end

end

    
    