function [avg_err, dx, dy] = translationEstimation(ref, newR, newBad, C)
    %% constants
    global PM_MAX_RANGE
    global PM_MIN_RANGE
    global PM_MAX_ERR
    %%
%     hi1 = 0; hi2 = 0; hwi1 = 0; hwi2 = 0; hw1= 0; hw2 = 0; hwh11 = 0;
%     hwh12 = 0; hwh21 = 0; hwh22 = 0; w = 0;
%     dr = 0;
%     abs_err = 0;
%     n = 0;
%     for i = 1:min(size(ref.data,1),size(newR,1))
%         dr = ref.data(i,2) - newR(i);
%         abs_err = abs_err + abs(dr);
%         if( ~ref.bad(i) && ~newBad(i) && newR(i) < PM_MAX_RANGE && newR(i) > PM_MIN_RANGE && abs(dr) < PM_MAX_ERR)
%             w = C/ (dr*dr + C);
%             n = n + 1;
% 
%             hi1 = cos(ref.data(i,1));
%             hi2 = sin(ref.data(i,1));
% 
%             hwi1 = hi1*w;
%             hwi2 = hi2*w;
% 
%             hw1 = hw1 + hwi1*dr;
%             hw2 = hw2 + hwi2*dr;
% 
%             hwh11 = hwh11 + hwi1*hi1;
%             hwh12 = hwh12 + hwi2*hi2;
%             hwh21 = hwh21 + hwi2*hi1;
%             hwh22 = hwh22 + hwi2*hi2;
% 
%         end
%     end
%     D = 0;
%     D = hwh11*hwh22-hwh12*hwh21;
%     inv11 = hwh22/D;
%     inv12 = -hwh12/D;
%     inv21 = -hwh12/D;
%     inv22 = hwh11/D;
%     dx = inv11*hw1+inv12*hw2;
%     dy = inv21*hw1+inv22*hw2;
%     avg_err = abs_err/n;
%%
      deltaR = ref.data(:,2) - newR;
      avg_err = mean(abs(deltaR));
      I = ~(ref.bad == 0 & newBad == 0 & abs(deltaR) < PM_MAX_ERR);
      tmpAngle = ref.data(:,1);
      deltaR(I) = [];
      tmpAngle(I) = [];
      w = C ./ ((deltaR .^ 2) + C);
      W = diag(w);
      H = [ cos(tmpAngle), sin(tmpAngle)];
      tmp = inv(H' * W * H);
      d = tmp * H' * W * deltaR;
      dx = d(1);
      dy = d(2);
      

end
