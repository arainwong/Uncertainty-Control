function newArray = removeZero(oldArray)
%%%
% remove zero elements from oldArray
% 
% Input requirement:
% oldArray: n * n
% 
%%%

    oldArray(oldArray==0) = [];
    newArray = oldArray;

end

