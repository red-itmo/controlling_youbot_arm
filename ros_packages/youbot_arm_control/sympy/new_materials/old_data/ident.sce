r = read("~/ident_data/data_5.txt", -1, 16)
subplot(2, 2, 1)
for i=1:5
    plot2d(r(:,16), r(:,i), i)
end
subplot(2, 2, 2)
for i=6:10
    plot2d(r(:,16), r(:,i), i-5)
end
subplot(2, 2, 3)
for i=11:15
    plot2d(r(:,16), r(:,i), i-10)
end

