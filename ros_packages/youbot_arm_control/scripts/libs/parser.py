#import roslib; roslib.load_manifest('urdfdom_py')

import rospy

from urdf_parser_py.urdf import URDF

#robot = URDF.from_xml_string("<robot name='myrobot'></robot>")
robot = URDF.from_xml_file('/media/data/evo/robotics_report/ros_packages/youbot_arm_control/urdf/youbot_arm_only_dh.urdf.xacro')
#robot = URDF.from_parameter_server()

print(robot)


def inverse(self, xyz, rpy, hh):
      qs = []
      # hh = self.round_rad(hh)

      px, py, pz = hh[:3, 3]
      r = hh[:3, :3]

      xt = r[:3, 0]
      yt = r[:3, 1]
      zt = r[:3, 2]

      pxy = math.sqrt(px ** 2 + py ** 2)

      # a method of projection a given general goal orientation
      #   into the subspace of KUKAYoubot' arm
      m = scipy.dot(1 / pxy, [-py, px, 0])
      # m = [0,-1,0]
      k = scipy.cross(zt, m)
      pzt = scipy.cross(m, k)
      cost = scipy.dot(zt, pzt)
      sint = scipy.dot(scipy.cross(pzt, zt), k)

      pyt = scipy.dot(cost, yt) + scipy.dot(sint, scipy.cross(yt, k)) + \
            scipy.dot(scipy.dot((1 - cost), scipy.dot(k, yt)), k)
      pxt = scipy.cross(pyt, pzt)
      pr = scipy.transpose([pxt, pyt, pzt])

      # kinematic equations
      q1 = math.atan2(py, px)
      c1 = math.cos(q1)
      s1 = math.sin(q1)
      q5 = math.atan2(pr[0, 0] * s1 - pr[1, 0] * c1,
                      pr[0, 1] * s1 - pr[1, 1] * c1)

      s234 = pr[0, 2] * c1 + pr[1, 2] * s1
      c234 = -pr[2, 2]
      q234 = math.atan2(s234, c234)
      q234 -= math.pi / 2

      px24 = px * math.cos(q1) + py * math.sin(q1) - self.d[5] * \
                                                     (pr[0, 2] * math.cos(q1) +
                                                      pr[1, 2] * math.sin(q1)) - \
             self.a[1]
      py24 = px * math.sin(q1) - py * math.cos(q1) - self.d[5] * pr[0, 2] * \
                                                     math.sin(q1) + self.d[5] * \
                                                                    pr[
                                                                          1, 2] * math.cos(
            q1)
      pz24 = pz - self.d[0] - self.d[1] - self.d[5] * pr[2, 2]
      print px24, py24, pz24
      print px, py, pz
      h = px24 ** 2 + py24 ** 2 + pz24 ** 2 - self.a[2] ** 2 - self.a[3] ** 2
      print h
      # algebraic approach
      # h = (px * c1 + py * s1 - self.a[1] - self.d[5] * s234)**2 + \
      #     (pz - self.d[0] - self.d[1] - self.d[5] * c234)**2 - \
      #     self.a[2]**2 - self.a[3]**2
      cosq3 = h / (2 * self.a[2] * self.a[3])

      print 'h, 2*a3+a4', h, 2 * self.a[2] * self.a[3]
      print cosq3
      q3 = math.atan2(math.sqrt(1 - cosq3 ** 2), cosq3)

      # algebraic approach
      # s3 = math.sin(q3)
      # c3 = math.cos(q3)
      # s2 = (c3 * self.a[1]) * (pz - s234 * self.a[3]) - \
      #      s3 * self.a[2] * (px * c1 + py * s1 - c234 * self.a[3]) / \
      #      (c3 * self.a[2] + self.a[1]) ** 2 + s3 ** 2 * self.a[2] ** 2
      # c2 = (c3 * self.a[1]) * (px * c1 + py * s1 - c234 * self.a[3]) + \
      #      s3 * self.a[2] * (pz - s234 * self.a[3]) / (c3 * self.a[2] +
      #      self.a[1]) ** 2 + s3 ** 2 * self.a[2] ** 2
      # q2 = math.atan2(s2, c2)

      # geometric approach
      q2 = math.atan2(math.sqrt(px24 ** 2 + py24 ** 2), pz24) - \
           math.atan2(self.a[3] * math.sin(q3),
                      self.a[2] + self.a[3] * math.cos(q3))

      # TODO AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaaa!
      # q2 = math.pi/2 - q2

      q4 = q234 - q2 - q3

      return q1, q2, q3, q4, q5






fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)

t = np.arange(0.0, 1.0, 0.001)

a0 = 5
f0 = 3
s = a0 * np.sin(2 * np.pi * f0 * t)

l, = plt.plot(t, s, lw=2, color='red')
plt.axis([0, 1, -10, 10])

axcolor = 'lightgoldenrodyellow'
axfreq = plt.axes([0.25, 0.1, 0.65, 0.03])
axamp = plt.axes([0.25, 0.15, 0.65, 0.03])

sfreq = Slider(axfreq, 'Freq', 0.1, 30.0, valinit=f0)
samp = Slider(axamp, 'Amp', 0.1, 10.0, valinit=a0)


def update(val):
    amp = samp.val
    freq = sfreq.val
    l.set_ydata(amp * np.sin(2 * np.pi * freq * t))
    fig.canvas.draw_idle()


sfreq.on_changed(update)
samp.on_changed(update)

resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


def reset(event):
    sfreq.reset()
    samp.reset()


button.on_clicked(reset)

rax = plt.axes([0.025, 0.5, 0.15, 0.15])
radio = RadioButtons(rax, ('red', 'blue', 'green'), active=0)


def colorfunc(label):
    l.set_color(label)
    fig.canvas.draw_idle()


radio.on_clicked(colorfunc)

plt.show()

plt.xlim(-m, m)
    plt.ylim(-m, m)
    plt.hlines(0, -m, m)
    plt.vlines(0, -m, m)
    plt.grid(True)

    plt.plot((0, 0), (0, -10), 'g', (0, 0), (0, -10), 'bo', linewidth=3.0)
    plt.plot(x, y, 'g', x, y, 'bo', linewidth=3.0)
    plt.title("Sets angles")
    plt.plot(point[0], point[2], 'ro')
    plt.plot(point_gripper[0], point_gripper[2], 'mo')