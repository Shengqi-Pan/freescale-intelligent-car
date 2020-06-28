void is_ring()
{
    if(sensor[0] > 1.1*induc_ref[0] && sensor[1] > 1.1*induc_ref[1])
        if((ad[3] - ad[4]) > 0 ? ad[3] - ad[4] : ad[4] - ad[3]) > 80) //两个电感差的绝对值大于80
            P52 = 0;

}