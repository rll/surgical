stringBase = 'suture1_';
stringExt = '.png';
baseLength = length(stringBase);
extLength = length(stringExt);

for camNum=1:3
    files = dir ([stringBase num2str(camNum) '*' stringExt]);
    %c++ code requires one image just to pull image info from
    currName = files(1).name;
    indDash = findstr(currName, '-');
    reName = ['./renamed/' currName(1:indDash) num2str(0) stringExt];
    copyfile(currName, reName);

    renameNum = 1;
    %first, get ims with 1 di
    for digitNums = 1:4
        for file = 1:length(files)
            currName = files(file).name;
            if (length(currName) == baseLength+2+extLength+digitNums)
                indDash = findstr(currName, '-');
                reName = ['./renamed/' currName(1:indDash) num2str(renameNum) stringExt];
                %currName
                %reName
                copyfile(currName, reName);                
                renameNum = renameNum+1;
            end
        end
    end
    
end