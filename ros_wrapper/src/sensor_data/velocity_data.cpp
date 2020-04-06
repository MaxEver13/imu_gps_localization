/*
 * @Description: 同步velocity数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-05
 */

#include "sensor_data/velocity_data.h"

namespace ImuGpsLocalization {

bool VelocityData::SyncData(std::deque<VelocityDataPtr>& unSyncedData, std::deque<VelocityDataPtr>& syncedData, double sync_time) {
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做插值
  while (unSyncedData.size() >= 2) {
    // 如果buffer中第一帧数据比要同步的时间还晚，表明同步时间前面没有数据，无从插值，退出
    if (unSyncedData.front()->timestamp > sync_time) 
      return false;

    // 到这里，说明同步时间前已经有一帧数据
    // 如果buffer中第二帧数据也比同步时间早，那么把第一帧数据丢掉，继续往后找
    if (unSyncedData.at(1)->timestamp < sync_time) {
      unSyncedData.pop_front();
      continue;
    }

    // 到这里，说明同步时间已经处于前两帧数据之间
    // 如果同步时间与第一帧数据时间相差太远，说明中间丢包了，不适合插值
    // 但是第二帧数据可能作为下一次同步时间的前一个数据, 把第一帧数据丢掉，退出
    if (sync_time - unSyncedData.front()->timestamp > 0.2) {
      unSyncedData.pop_front();
      return false;
    }
    
    // 如果同步时间与第二帧数据时间相差太远，也是丢包了，不适合插值
    // 但是第二帧数据可能作为下一次同步时间的前一个数据
    // 只丢掉第一个数据，退出
    if (unSyncedData.at(1)->timestamp- sync_time > 0.2) {
      unSyncedData.pop_front();
      return false;
    }
    break;
  }
  // 保证buffer前两帧是插值需要用的数据
  if (unSyncedData.size() < 2)
    return false;

  VelocityDataPtr front_data_ptr = unSyncedData.at(0);
  VelocityDataPtr back_data_ptr = unSyncedData.at(1);
  VelocityDataPtr synced_data_ptr = std::make_shared<VelocityData>();  

  double front_scale = (back_data_ptr->timestamp - sync_time) / (back_data_ptr->timestamp - front_data_ptr->timestamp);
  double back_scale = (sync_time - front_data_ptr->timestamp) / (back_data_ptr->timestamp - front_data_ptr->timestamp);
  synced_data_ptr->timestamp = sync_time;
  synced_data_ptr->vel(0) = front_data_ptr->vel(0) * front_scale + back_data_ptr->vel(0) * back_scale;
  synced_data_ptr->vel(1) = front_data_ptr->vel(1) * front_scale + back_data_ptr->vel(1) * back_scale;
  synced_data_ptr->vel(2) = front_data_ptr->vel(2) * front_scale + back_data_ptr->vel(2) * back_scale;

  syncedData.push_back(synced_data_ptr);

  return true;
}


}