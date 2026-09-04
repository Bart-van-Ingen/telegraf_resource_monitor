# Installing Telegraf

Telegraf can be fetched automatically into the workspace by the `telegraf_vendor` package
(recommended), or be installed system-wide (apt or binary, below).

## Through the vendor package (recommended)

Nothing to do here: `telegraf_vendor` downloads telegraf during `colcon build`
and installs it into the workspace, so no system-wide install is needed. If
telegraf is already on your system, it is reused instead of downloaded. See
[docs/vendor_packages.md](docs/vendor_packages.md) for what a vendor package is
and how this one works. Skip straight to [Installing the Package](#installing-the-package).

<details>
<summary><b>Through apt</b></summary>

As per https://www.influxdata.com/get-telegraf/

```bash
# Add InfluxDB repository
# influxdata-archive.key GPG fingerprint:
#   Primary key fingerprint: 24C9 75CB A61A 024E E1B6  3178 7C3D 5715 9FC2 F927
#   Subkey fingerprint:      9D53 9D90 D332 8DC7 D6C8  D3B9 D8FF 8E1F 7DF8 B07E
wget -q https://repos.influxdata.com/influxdata-archive.key
gpg --show-keys --with-fingerprint --with-colons ./influxdata-archive.key 2>&1 | grep -q '^fpr:\+24C975CBA61A024EE1B631787C3D57159FC2F927:$' && cat influxdata-archive.key | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/influxdata-archive.gpg > /dev/null
echo 'deb [signed-by=/etc/apt/trusted.gpg.d/influxdata-archive.gpg] https://repos.influxdata.com/debian stable main' | sudo tee /etc/apt/sources.list.d/influxdata.list

sudo apt-get update && sudo apt-get install telegraf
```

</details>

<details>
<summary><b>As linux binary</b></summary>

find the specific version number from the [telegraf release page](https://github.com/influxdata/telegraf/releases) in the format x.xx.x.

Then fill this value accordingly with the following commands in terminal

```bash
wget https://dl.influxdata.com/telegraf/releases/telegraf-x.xx.x_linux_amd64.tar.gz \
    && tar -xzf telegraf-x.xx.x_linux_amd64.tar.gz \
    && rm telegraf-x.xx.x_linux_amd64.tar.gz \
    && mv telegraf-x.xx.x/usr/bin/telegraf /usr/local/bin/telegraf \
    && chmod +x /usr/local/bin/telegraf
```

</details>

