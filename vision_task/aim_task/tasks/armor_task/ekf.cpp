#include "ekf.hpp"
#include <iostream>
#include <numeric>

namespace armor_task
{

// 鐘舵€佸悜閲忥細(xc,vxc,yc,vyc,zc,vyz,yaw,w,r,dl,dh)
// a: angle
// w: angular velocity
// dl: r2 - r1
// dh: z2 - z1
/*
鍏朵腑涓嬫爣c琛ㄧず鏃嬭浆涓績锛堝嵆灏忚溅涓績锛夌殑鐗╃悊閲忋€倅aw涓鸿鐢叉澘鐩稿浜庝笘鐣屽潗鏍囩郴鍘熺偣鐨勫亸鑸锛寃涓烘棆杞殑瑙掗€熷害锛宺涓烘棆杞崐寰勶紝dl涓洪暱鐭酱
 宸紝dh涓洪珮搴﹀樊銆傚紩鍏l,dh鐨勫師鍥犳槸锛屽埗浣滆鑼冨彧瑕佹眰灏忚溅涓績瀵圭О浣嶇疆鐨勮鐢叉澘楂樺害涓庡崐寰勭浉鍚岋紝鑰岀浉閭昏鐢叉澘鏃嬭浆鍗婂緞鍜屽畨瑁呴珮搴﹀彲鑳戒笉鍚屻€?
*/

// 瑙傛祴鍚戦噺锛氾紙yaw,pitch,distance,orientation_yaw锛?
/*
姝aw,pitch闈炲郊yaw,pitch銆傚湪瑙傛祴鍚戦噺寤烘ā鏃讹紝鎴戜滑鍙栦笘鐣屽潗鏍囩郴鍘熺偣寤虹珛鏋佸潗鏍囩郴锛屾澶勭殑yaw鏄洰鏍囧湪鏋佸潗鏍囩郴涓嬫按骞抽潰鐨勮搴︼紝pitch鏄洰鏍囧湪鏋佸潗鏍囩郴涓嬪瀭鐩撮潰鐨勮搴︼紝distance鏄洰鏍囦笌鐩告満鐨勮窛绂?orieantaion_yaw鏄洰鏍囩浉瀵逛簬鍘熺偣鐨勫亸鑸
*/
Ekf::Ekf(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add) :
    x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["nis_fail"] = 0.0;
    data["nees_fail"] = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd Ekf::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q)
{
    return predict(F, Q, [&](const Eigen::VectorXd &x) { return F * x; });
}

Eigen::VectorXd Ekf::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
}

Eigen::VectorXd Ekf::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    return update(z, H, R, [&](const Eigen::VectorXd &x) { return H * x; }, z_subtract);
}

Eigen::VectorXd Ekf::update(const Eigen::VectorXd &z,
                            const Eigen::MatrixXd &H,
                            const Eigen::MatrixXd &R,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    Eigen::VectorXd x_prior = x;
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Stable Compution of the Posterior Covariance
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    x = x_add(x, K * z_subtract(z, h(x)));

    /// 鍗℃柟妫€楠?
    Eigen::VectorXd residual = z_subtract(z, h(x));

    // 鏂板妫€楠?
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    double nis = residual.transpose() * S.inverse() * residual;
    double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

    // 鍗℃柟妫€楠岄槇鍊硷紙鑷敱搴?4锛屽彇缃俊姘村钩95%锛?
    constexpr double nis_threshold = 0.711;
    constexpr double nees_threshold = 0.711;

    if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
    if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
    total_count_++;
    last_nis = nis;

    recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

    if (recent_nis_failures.size() > window_size)
    {
        recent_nis_failures.pop_front();
    }

    int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

    data["residual_yaw"] = residual[0];
    data["residual_pitch"] = residual[1];
    data["residual_distance"] = residual[2];
    data["residual_angle"] = residual[3];
    data["nis"] = nis;
    data["nees"] = nees;
    data["recent_nis_failures"] = recent_rate;

    return x;
}
// #include <Eigen/Dense>
// #include <functional>
// #include <deque>
// #include <map>
// #include <string>
// #include <iostream>
#include <numeric>

// // 瀹氫箟甯搁噺缁村害锛屽埄鐢ㄦā鏉垮弬鏁版垨鐩存帴瀹氫箟鍙互鍚敤缂栬瘧鏈熶紭鍖?
// constexpr int STATE_DIM = 11; // 鐘舵€佸悜閲忕淮搴?
// constexpr int MEAS_DIM = 4;   // 瑙傛祴鍚戦噺缁村害

// // 绫诲瀷鍒悕锛屾柟渚夸功鍐?
// using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
// using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
// using MeasVector  = Eigen::Matrix<double, MEAS_DIM, 1>;
// using MeasMatrix  = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;
// using MeasNoise   = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;

// class Ekf
// {
// public:
//     // 鏋勯€犲嚱鏁?
//     Ekf(const StateVector &x0, const StateMatrix &P0);

//     // 棰勬祴姝ラ
//     StateVector predict(const StateMatrix &F, const StateMatrix &Q);

//     // 鏇存柊姝ラ
//     StateVector update(const MeasVector &z, 
//                        const MeasMatrix &H, 
//                        const MeasNoise &R,
//                        std::function<MeasVector(const StateVector &)> h_func,
//                        std::function<MeasVector(const MeasVector &, const MeasVector &)> z_subtract_func);

//     // 鏁版嵁鎺ュ彛
//     std::map<std::string, double> data;

//     // 澶栭儴璁剧疆鐘舵€佸姞娉曞嚱鏁帮紙澶勭悊瑙掑害褰掍竴鍖栫瓑锛?
//     std::function<StateVector(const StateVector &, const StateVector &)> x_add_func;

// private:
//     StateVector x;
//     StateMatrix P;
    
//     // 缂撳瓨鍗曚綅鐭╅樀锛岄伩鍏嶉噸澶嶅垱寤?
//     const StateMatrix I = StateMatrix::Identity();

//     // 婊戝姩绐楀彛鐩稿叧鍙橀噺
//     std::deque<int> recent_nis_failures;
//     int nis_failure_sum = 0; // 缁存姢涓€涓拰锛岄伩鍏嶆瘡娆￠亶鍘?
//     const size_t window_size = 50; // 鍋囪绐楀彛澶у皬涓?0锛屽彲璋冩暣

//     long long total_count_ = 0;
//     long long nis_count_ = 0;
//     long long nees_count_ = 0;
// };

// // ================= 瀹炵幇閮ㄥ垎 (閫氬父鏀惧湪 .cpp) =================

// Ekf::Ekf(const StateVector &x0, const StateMatrix &P0)
//     : x(x0), P(P0)
// {
//     // 鍒濆鍖?map锛岄伩鍏嶈繍琛屾椂鎻掑叆寮€閿€
//     data["residual_yaw"] = 0.0;
//     data["residual_pitch"] = 0.0;
//     data["residual_distance"] = 0.0;
//     data["residual_angle"] = 0.0;
//     data["nis"] = 0.0;
//     data["nees"] = 0.0;
//     data["nis_fail"] = 0.0;
//     data["nees_fail"] = 0.0;
//     data["recent_nis_failures"] = 0.0;
// }

// StateVector Ekf::predict(const StateMatrix &F, const StateMatrix &Q)
// {
//     // 榛樿棰勬祴妯″瀷锛氱嚎鎬?x = F * x
//     // 1. 鍗忔柟宸娴? P = F * P * F^T + Q
//     // 涓轰簡鏋佽嚧鎬ц兘锛孍igen 浼氬杩欑鍥哄畾澶у皬鐭╅樀鍋氬睍寮€浼樺寲
//     P = F * P * F.transpose() + Q;
    
//     // 2. 鐘舵€侀娴?
//     x = F * x;
    
//     return x;
// }

// StateVector Ekf::update(const MeasVector &z,
//                         const MeasMatrix &H,
//                         const MeasNoise &R,
//                         std::function<MeasVector(const StateVector &)> h_func,
//                         std::function<MeasVector(const MeasVector &, const MeasVector &)> z_subtract_func)
// {
//     StateVector x_prior = x; // 淇濆瓨鍏堥獙鐢ㄤ簬璁＄畻 NEES

//     // 1. 璁＄畻娈嬪樊 y = z - h(x)
//     MeasVector z_pred = h_func(x);
//     MeasVector y = z_subtract_func(z, z_pred);

//     // 2. 璁＄畻娈嬪樊鍗忔柟宸?S = H * P * H^T + R
//     // 浣跨敤 noalias() 鍛婅瘔 Eigen 杩欓噷娌℃湁娣峰彔锛屽彲浠ョ洿鎺ヨ绠楃粨鏋?
//     MeasNoise S = H * P * H.transpose() + R;

//     // 3. 璁＄畻鍗″皵鏇煎鐩?K = P * H^T * S^-1
//     // 浼樺寲锛氫娇鐢?LDLT 鍒嗚В姹傝В绾挎€ф柟绋嬶紝姣?.inverse() 蹇笖鏁板€兼洿绋冲畾
//     // K * S = P * H^T  =>  K = (P * H^T) / S
//     Eigen::Matrix<double, STATE_DIM, MEAS_DIM> P_Ht = P * H.transpose();
//     Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = S.ldlt().solve(P_Ht.transpose()).transpose();

//     // 4. 鏇存柊鐘舵€?x = x + K * y
//     // 闇€瑕佷娇鐢?x_add_func 澶勭悊瑙掑害鍔犳硶锛堝鏈夛級
//     StateVector dx = K * y;
//     if (x_add_func) {
//         x = x_add_func(x, dx);
//     } else {
//         x += dx;
//     }

//     // 5. 鏇存柊鍗忔柟宸?P = (I - K * H) * P
//     // 浣跨敤鏍囧噯褰㈠紡閫熷害鏈€蹇€傚鏋滄暟鍊肩ǔ瀹氭€ф瀬宸紝鎵嶉渶瑕佺敤 Joseph Form銆?
//     // 杩欓噷浣跨敤鏍囧噯褰㈠紡浼樺寲鐗堬細P -= K * H * P
//     // 鎴栬€呮洿绋冲仴鐨? P = (I - K*H) * P
//     StateMatrix I_KH = I - K * H;
//     P = I_KH * P * I_KH.transpose() + K * R * K.transpose(); // Joseph Form (铏界劧鎱㈢偣锛屼絾淇濊瘉P瀵圭О姝ｅ畾锛屽闀挎椂闂磋繍琛屽緢閲嶈)

//     // --- 浠ヤ笅鏄粺璁′笌璋冭瘯閮ㄥ垎 (宸蹭紭鍖? ---
    
//     // 璁＄畻 NIS: y^T * S^-1 * y
//     // 鍒╃敤涔嬪墠鍒嗚В杩囩殑 S 鍔犻€熸眰瑙?
//     double nis = y.transpose() * S.ldlt().solve(y);
    
//     // 璁＄畻 NEES: (x - x_prior)^T * P^-1 * (x - x_prior)
//     StateVector err_x = x - x_prior; // 杩欓噷绠€鍗曞噺娉曪紝濡傛灉鏄搴﹂渶澶勭悊
//     double nees = err_x.transpose() * P.ldlt().solve(err_x);

//     constexpr double nis_threshold = 7.779; // 鑷敱搴?锛?0%缃俊鍖洪棿 (0.711澶皬浜嗭紝寤鸿鏌ヨ〃)
//     constexpr double nees_threshold = 19.675; // 鑷敱搴?1

//     int current_fail = 0;
//     if (nis > nis_threshold) {
//         nis_count_++;
//         data["nis_fail"] = 1.0;
//         current_fail = 1;
//     } else {
//         data["nis_fail"] = 0.0;
//     }

//     if (nees > nees_threshold) {
//         nees_count_++;
//         data["nees_fail"] = 1.0;
//     } else {
//         data["nees_fail"] = 0.0;
//     }

//     total_count_++;

//     // O(1) 婊戝姩绐楀彛璁＄畻
//     recent_nis_failures.push_back(current_fail);
//     nis_failure_sum += current_fail;

//     if (recent_nis_failures.size() > window_size) {
//         int removed = recent_nis_failures.front();
//         recent_nis_failures.pop_front();
//         nis_failure_sum -= removed;
//     }

//     data["residual_yaw"] = y[0];
//     data["residual_pitch"] = y[1];
//     data["residual_distance"] = y[2];
//     data["residual_angle"] = y[3];
//     data["nis"] = nis;
//     data["nees"] = nees;
//     data["recent_nis_failures"] = static_cast<double>(nis_failure_sum) / recent_nis_failures.size();

//     return x;
// }
}  // namespace armor_task


