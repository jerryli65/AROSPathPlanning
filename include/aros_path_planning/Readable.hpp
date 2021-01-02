#ifndef AROSPATHPLANNING_READABLE_HPP
#define AROSPATHPLANNING_READABLE_HPP
namespace aros::Readable{
    /// class for initializing the robot chassis as a variable
    class Readable{
    public:
        ///Returns x position of A&M logo
        [[nodiscard]] float x_position();
        ///Returns y position of A&M logo
        [[nodiscard]] float y_position();
        ///Returns orientation to scoreboard
        [[nodiscard]] float orientation();
    };
}
#endif