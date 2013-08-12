% n = 7;
% r = 4;
% 
% A = -1*ones(r, n+1);
% 
% for i = 0:r-1,
%     for j = 0:2*n
%         if (n-j) >= i,
%         temp = 1;
%         for k = 0:i-1,
%             temp = temp*(n-k-j)
%         end
%         A(i+1, j+1) = n-j-i;
%         end
%     end
% end
% 
% A


% n = 5;
% r = 3;
% 
% A = -1*ones(r, 2*(n+1));
% 
% for i = 0:r-1,
%     for j = 0:2*(n+1)
%         if (n-j) >= i && j <= n,
%         temp = 1;
%         for k = 0:i-1,
%             temp = temp*(n-k-j);
%         end
%         A(i+1, j+1) = temp;
%         
%         elseif (n-(j-n-1)) >= i && j > n,
%             temp = 1;
%             for k = 0:i-1,
%                 temp = temp*(n-k- (j-n-1));
%             end
%             %A(i+1, j+1) = n- (j-n-1) -i;
%             A(i+1, j+1) = temp;
%         end
%     end
% end
% 
% A

n = 9;
r = 5;

A = -1*ones(2*r, 2*(n+1));

for i = 0:2*r-1,
    for j = 0:2*(n+1)
        if (n-j) >= i && j <= n && i < r,
        temp = 1;
        for k = 0:i-1,
            temp = temp*(n-k-j);
        end
        %A(i+1, j+1) = temp;
        A(i+1, j+1) = n-j-i;

        elseif (n-(j-n-1)) >= (i-r) && j > n && i >= r,
            temp = 1;
            for k = 0:(i-r)-1,
                temp = temp*(n-k- (j-n-1));
            end
            A(i+1, j+1) = n- (j-n-1) -(i-r);
            
            %A(i+1, j+1) = temp;
        end
    end
end

A