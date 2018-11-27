function [ds] = template_import_XDatav2(csvFile, reMapFile, nameDS)
% This function generates a .mat variable from the csvFile and uses the reMapFile to simplify the names
% csvFile: File generated using rosbag2csv. The file-path most be used.
% reMapFile: config File (with path) with the special re-maping for the csv fields
% nameDS: string for the output name for the .mat structure containing all the fields
% defined in reMap.txt file
% TUM ICS
% Emmanuel Dean dean@tum.de
% Florian Bergner florian.bergner@tum.de


ds = [];

fprintf('reading reMap file: "%s" ...\n',reMapFile);
fileId = fopen(reMapFile);
if fileId == -1
    error('File %s does not exist',reMapFile);
end


line = fgetl(fileId);

fields={};
fieldNames={};

while line~=-1
%     fprintf('Line: %s\n',line);
%     fprintf('Char %c\n',line(1));
    if line(1)=="#"
%         fprintf('Skipped line\n');
        line = fgetl(fileId);
        continue
    end
    aux = textscan(line,'%s','Delimiter',':');
       
    fields={fields{1:end},aux{1}{1}};
    fieldNames={fieldNames{1:end},aux{1}{2}};

    line = fgetl(fileId);
end
fclose(fileId);

fprintf('import "%s" ...\n',csvFile);

fileId = fopen(csvFile);

if fileId == -1
    error('File %s does not exist',csvFile);
end

colheadersStr = fgetl(fileId);
colheaders = textscan(colheadersStr,'%s','Delimiter',',');
colheaders = colheaders{1};

formatSpec = repmat('%s',[1,length(colheaders)-1]);
formatSpec = strcat(formatSpec,'%[^\n\r]');

textdata = textscan(fileId,formatSpec,'Delimiter',',','HeaderLines',0,'ReturnOnError',false);

fclose(fileId);

% read fields from csv and create cells with proper names
for k=1:length(colheaders)
    for l=1:length(fields)      
        % found field
        if(strcmp(fields{l},colheaders{k}))
            name = fieldNames{l};
            strVals = textdata{k};
            
            % the header timestamp has to be handled differently
            if(strcmp(fields{l},'field.header.stamp'))
                temp = uint64(zeros(length(strVals),1));
                for m=1:length(strVals)
                    temp(m) = str2num(sprintf('uint64(%s)',strVals{m}));
                end
                ds.(fieldNames{l}) = temp;
            else
                ds.(fieldNames{l}) = str2double(strVals);  
            end
            fprintf(1,'---> got %s\n',name);       
        end
    end
end   

[mName,errmsg]=sprintf('%sDS',nameDS);
[mNameExt,errmsg]=sprintf('%sDS.mat',nameDS);
ds.name=mName;

eval(strcat(mName,' = ds'));

save(mNameExt,mName);

end
