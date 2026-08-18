/* Test application for GSC/DOA.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.  See the License for the
   specific language governing permissions and limitations under the License.
*/

#include "unity.h"
#include <stdio.h>
#include <string.h>

extern "C" void app_main(void)
{
    /* This function will not return, and will be busy waiting for UART input.
     * Make sure that task watchdog is disabled if you use this function.
     */
    unity_run_menu();
}
