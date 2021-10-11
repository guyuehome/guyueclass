import java.io.*;

class DataReader
{
public DataReader(DataInput data_input)
{
   m_data_input = data_input;
}

public byte[] readBuffer(int length)
{
   byte[] buffer = new byte[length];

   try
   {
       m_data_input.readFully(buffer, 0, length);
   }

   catch (StreamCorruptedException e)
   {
       System.out.println("Stream Corrupted Exception Occured");
       buffer = new byte[0];
   }
   catch (EOFException e)
   {
       System.out.println("EOF Reached");
       buffer = new byte[0];
   }
   catch (IOException e)
   {
       System.out.println("IO Exception Occured");
       buffer = new byte[0];
   }

   return buffer;
}

private DataInput m_data_input;
}
