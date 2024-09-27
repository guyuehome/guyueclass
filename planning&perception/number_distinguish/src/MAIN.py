import torch
import torch.nn as nn
from torch.autograd import Variable
from include import IMG_h
from include import CNN_h
from include import UI_h
import torch.utils.data as Data

train_path = 'D:/PycharmProjects/Num_distinguish/train_data/labels.txt'
test_path = 'D:/PycharmProjects/Num_distinguish/test_data'

train_data = CNN_h.MyDataset(train_path)
train_loader = Data.DataLoader(dataset=train_data, batch_size=1, shuffle=True, num_workers=0)

cnn = CNN_h.CNN()
optimizer = torch.optim.Adam(cnn.parameters(), lr=0.001)
loss_func = nn.CrossEntropyLoss()

for i in range(10):
    for step, (x, y) in enumerate(train_loader):
        real_x = Variable(x)
        real_y = Variable(y)
        pre_y = cnn(real_x)
        loss = loss_func(pre_y, real_y)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        #print('loss:', float(loss.data))


img_dist = IMG_h.Carlicense_distinguish()
img_dist.carlicense_distinguish()


test_data = CNN_h.predicted_data(test_path)

decode_input = []
for i in range(len(test_data)):
    pre_test = cnn(test_data[i])
    decode_input.append(int(torch.max(pre_test,1)[1].data.numpy().squeeze()))

print(CNN_h.decode_output(decode_input))
UI_h.window(CNN_h.decode_output(decode_input))
