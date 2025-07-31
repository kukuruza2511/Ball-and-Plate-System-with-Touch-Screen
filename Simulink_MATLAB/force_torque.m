function [Fz, Ffx, Ffy] = force_torque(z_ball, z_plate, vz, vx, vy, Kpen, Dpen, mu, vthr)
    % Tính độ lún tiếp xúc
    penetration = z_ball - z_plate;
    if penetration < 0
        Fz = -Kpen * penetration - Dpen * vz; % Phản lực
    else
        Fz = 0;
    end
    
    % Tính lực ma sát
    vx_slip = vx; % Tốc độ trượt theo x
    vy_slip = vy; % Tốc độ trượt theo y
    
    if abs(vx_slip) <= vthr
        mux = mu * vx_slip / vthr;
    else
        mux = sign(vx_slip) * mu;
    end
    Ffx = mux * Fz; % Lực ma sát theo x

    if abs(vy_slip) <= vthr
        muy = mu * vy_slip / vthr;
    else
        muy = sign(vy_slip) * mu;
    end
    Ffy = muy * Fz; % Lực ma sát theo y
end