std::vector<float> _filter_laser_ranges(std::vector<float> laser_ranges)
{
    std::vector<float> filtered_ranges;
    int   i    = 1;
    int   cl   = laser_ranges.size() -1;
    bool  de   = false;
    float mean = 0;
    float a = 0;
    filtered_ranges.push_back(0);

    while(i < cl)
    {
        if(fabs(laser_ranges[i-1] - laser_ranges[i]) < FILTER_THRESHOLD)
        {
            de = true;
            do
            {
                if(fabs(laser_ranges[i+1] - laser_ranges[i]) < FILTER_THRESHOLD)
                {
                    mean = 0;
                    for(int k= -1; k < 2; k++)
                        mean+= laser_ranges[i+k];
                    a = (mean / 3.0);
                    filtered_ranges.push_back(a);
                    i++;
                }
                else
                {
                    filtered_ranges.push_back(laser_ranges[i]);
                    i++;
                    if( i < laser_ranges.size() - 1)
                    {
                        filtered_ranges.push_back(laser_ranges[i]);
                        i++;
                    }
                    de = false;
                }
            }while( de  && i < cl);
        }
        else
        {
            filtered_ranges.push_back(laser_ranges[i]);
            i++;
        }
    }
    filtered_ranges.push_back(laser_ranges[i]);
    
    return filtered_ranges;
}
