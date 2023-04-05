#include <stdio.h>
#include <stdint.h>

#define max_step_value 25000
#define max_degree_value 180

float map_func(float value, float s_min, float s_max, float d_min, float d_max)
{
    float s_out = s_max - s_min;
    float d_out = d_max - d_min;
    //float t_out = value * d_out / s_out;
    float scale = (value - s_min) / s_out;
    float t_out = (d_min + scale * d_out );
    return t_out;
    
}

int main()
{
    float value, s_min, s_max, d_min, d_max;
    printf("enter value, from min value ,from max value , to min value, to max value");
    scanf("%f %f %f %f %f",&value , &s_min, &s_max, &d_min, &d_max );
    printf("value = %f , from min = %f , from max = %f , to min = %f , to max = %f \n", value, s_min, s_max, d_min, d_max);
    float out = map_func(value, s_min, s_max, d_min, d_max);
    printf("the output value is %f \n",out);

    return 0;
}
