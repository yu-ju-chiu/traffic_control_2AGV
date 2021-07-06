A=imread('clear_map.jpg');
B = imresize(A, 0.15);
br=rgb2gray(B);%轉成灰階圖
[m,n]=size(br);

 for i=1:1:m
    for j=1:1:n
       if br(i,j)>200
         br(i,j)=0;
       else
         br(i,j)=1;
       end
    end
 end
 
 imshow(br);


fid=fopen('clear_map.txt','wt'); %寫入的檔案，各函式後面有說明

 for i=1:1:m
    for j=1:1:n
       if j==n
         fprintf(fid,'%g\n',br(i,j));
       else
        fprintf(fid,'%g ',br(i,j));
       end
    end
 end
fclose(fid);