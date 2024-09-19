function rho = rho(h)
    % Assuming linear relationship using Standard Atmosphere Model
    rho = (0.9093-1.225)/3000*h + 1.225;
end