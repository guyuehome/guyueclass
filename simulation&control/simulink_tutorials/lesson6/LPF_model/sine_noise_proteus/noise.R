#生成波形数据部分
noise<-rnorm(500,mean=36*3.5,sd=36)
noise<-round(noise)
noise_H<-as.character(as.hexmode(noise))
#写入文件部分
write.table(noise_H,file='C:/Users/lenovo/Desktop/noise_H.txt', row.names =FALSE,col.names =FALSE, quote =FALSE)