 function angle = cos2th( C,a,b )

 % Numerical ���� ó��: ���� �� ���� ���̰� ������ �κ��� ���� �հ� ���ų� �ſ� ���� Ŭ ���
 if max([ C a b]) - ( sum([ C a b]) - max([ C a b]) ) >= 0 && max([ C a b]) - ( sum([ C a b]) - max([ C a b]) ) < 1e-5
     if max([ C a b ]) == C
         angle = pi;
     else
         angle = 0;
     end
 else
      angle = acos( (a^2 + b^2 - C^2) / (2*a*b) ); % �Ϲ� ���� ���
 end
     


