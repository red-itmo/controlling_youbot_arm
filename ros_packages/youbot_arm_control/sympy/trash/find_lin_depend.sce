clc();
clear;
N = 21;
for i = 1:N
    name = "/media/data/evo/robotics_report/ros_packages/youbot_arm_control/sympy/EE/EE" + string(i-1) + ".txt";
    r = read(name, -1, 5);
    EE1(:,:,i) = r'
end
s = size(EE1(:,:,N));
printf("Succuessfully read; size of matrices: %dx%d\n", s(1), s(2))


fd = mopen('/home/eagle/study/last_version/log_scilab', 'w');

global A B pos columns

function make_combination(n,m, num)
    global A B pos columns
    pos = pos + 1
    for i = num:n
        if the_column <= i then
            columns(1,pos) = i+1;
        else
            columns(1,pos) = i;
        end
        for j = 1:n_E
            A((5*j-4):(5*j),pos) = Eh(:,i,j)
            B((5*j-4):(5*j)) = aim_vect(:,j)
        end
        if m == 1 then
            if rank(A) >= rank([A,B]) then
                k = linsolve(A,-B);
                flag = %t;
                for ind = 1: max(size(k))
                    if abs(k(ind)) < 10*10^(-12) then
                        flag = %f
                    end
                end
                if flag then
                    printf("Column No %2d is dependent on cols No", the_column);
                    disp(columns);
                    printf("with koefficients");
                    disp(k');
                    printf("\n")

                    mputl(msprintf("Column No %2d is dependent on cols No", the_column), fd);
                    mputl(string(columns), fd);
                    mputl(printf("with koefficients"), fd);
                    mputl(string(k'), fd);
                    mputl('', fd);
                end
            end
        else
            make_combination(n, m-1, i+1);
        end
    end
    pos = pos - 1
endfunction


//values of regressor for several random values of q, dq, ddq
E = EE1;   


size_E = size(E);
n_dots = size_E(3);

nc = max(size(E(:,:,1)));

for i = 1:nc
    printf("checking of %d column:\n", i)
    mputl(printf("checking of %d column:\n", i), fd)
    //prepare helpful variables
    for k = 1:n_dots
        aim_vect(:,k) = E(:,i,k);
        ind = 1;
        for kk = 1:nc
            if kk <> i then
                Eh(:,ind,k) = E(:,kk,k)
                ind = ind + 1;
            end 
        end
    end
    
    
    //find linear dependence for i-th column
    for j = 4:nc//(nc-1)
        n_E = ceil(j/5) + 15
        A = zeros(n_E * 5, j);
        B = zeros(n_E * 5, 1);
        the_column = i;
        columns = zeros(1,j);
        pos = 0;
        make_combination((nc-1), j, 1)
        
        printf("Checked dependency on %d/%d columns\n", j, nc);
        mputl(printf("Checked dependency on %d/%d columns\n", j, nc), fd)
    end
end
