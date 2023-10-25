 function angle = cos2th( C,a,b )

 % Numerical 오차 처리: 가장 긴 변의 길이가 나머지 두변의 길이 합과 같거나 매우 조금 클 경우
 if max([ C a b]) - ( sum([ C a b]) - max([ C a b]) ) >= 0 && max([ C a b]) - ( sum([ C a b]) - max([ C a b]) ) < 1e-5
     if max([ C a b ]) == C
         angle = pi;
     else
         angle = 0;
     end
 else
      angle = acos( (a^2 + b^2 - C^2) / (2*a*b) ); % 일반 적인 경우
 end
     


