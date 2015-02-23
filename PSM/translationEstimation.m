function [avg_err, dx, dy] = translationEstimation(ref, newR, newBad, C)
    %% constants
    global PM_MAX_RANGE
    global PM_MIN_RANGE
    global PM_MAX_ERR
%%
      deltaR = ref.data(:,2) - newR;
      I = ~(ref.bad == 0 & newBad == 0 & abs(deltaR) < PM_MAX_ERR);
      tmpAngle = ref.data(:,1);
      deltaR(I) = [];
      tmpAngle(I) = [];
      avg_err = mean(abs(deltaR));
      w = C ./ ((deltaR .^ 2) + C);
      W = diag(w);
      H = [ cos(tmpAngle), sin(tmpAngle)];
      tmp = inv(H' * W * H);
      d = tmp * H' * W * deltaR;
      dx = d(1);
      dy = d(2);
      

end
