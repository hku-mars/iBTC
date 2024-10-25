function M = read_from_file(filename,cols)
fileID = fopen(filename);
A = fscanf(fileID, '%g');
B = A.';
M = reshape(B,cols, size(A,1)/cols).';
end