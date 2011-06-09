max_len = -1000;
for l = 1:length(lines)
   xy = [lines(l).point1; lines(l).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(l).point1 - lines(l).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
