/**

\author Michael Ferguson

@b matching of clouds for board localization.

**/

#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

/* 
 * TODO: new matching scheme
 *  
 *  what is working:
 *   evaluation using FitnessScore
 *   getting rigid transformation
 * 
 *  what is not working:
 *   icp fails to find a good solution
 */


