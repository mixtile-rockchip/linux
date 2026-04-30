    // SPDX-License-Identifier: GPL-2.0
    /*
    * Fixed YUV source fronted by a MAX9296 deserializer.
    *
    * User space is expected to configure the SerDes chain. This driver only
    * exposes a fixed CSI-2 source to Rockchip CIF/ISP.
    */

    #include <linux/i2c.h>
    #include <linux/delay.h>
    #include <linux/module.h>
    #include <linux/mutex.h>
    #include <linux/of.h>
    #include <linux/of_graph.h>
    #include <linux/rk-camera-module.h>
    #include <linux/slab.h>
    #include <linux/types.h>
    #include <linux/uaccess.h>

    #include <media/media-entity.h>
    #include <media/v4l2-async.h>
    #include <media/v4l2-ctrls.h>
    #include <media/v4l2-fwnode.h>
    #include <media/v4l2-subdev.h>

    #include <uapi/linux/rkcif-config.h>

    #define DRIVER_NAME "max9296_fixed_yuv"
    #define MAX9296_FIXED_WIDTH 1920
    #define MAX9296_FIXED_HEIGHT 1536
    #define MAX9296_FIXED_LINK_FREQ 750000000ULL
    #define MAX9296_FIXED_PIXEL_RATE 88473600ULL

struct max9296_serdes_reg {
    u8 addr;
    u8 page;
    u8 reg;
    u8 val;
    u16 delay_ms;
};

struct max9296_fixed_yuv {
    struct i2c_client *client;
        struct mutex lock;
        struct v4l2_subdev sd;
        struct media_pad pad;
        struct v4l2_mbus_framefmt fmt;
        struct v4l2_fwnode_endpoint bus_cfg;
        struct v4l2_ctrl_handler ctrl_handler;
        struct v4l2_ctrl *link_freq;
        struct v4l2_ctrl *pixel_rate;
        struct v4l2_fract frame_interval;
    bool streaming;
    bool serdes_configured;
    struct max9296_serdes_reg *max9296_seq;
    unsigned int max9296_seq_num;
    struct max9296_serdes_reg *max96717_seq;
    unsigned int max96717_seq_num;
    struct max9296_serdes_reg *isx031_seq;
    unsigned int isx031_seq_num;
    u32 module_index;
    const char *module_facing;
    const char *module_name;
    const char *lens_name;
};

static int max9296_fixed_s_stream(struct v4l2_subdev *sd, int on);

static const s64 max9296_fixed_link_freq_menu[] = {
    MAX9296_FIXED_LINK_FREQ,
};

static int max9296_fixed_parse_init_seq(struct device *dev,
                    struct device_node *np,
                    const char *prop_name,
                    struct max9296_serdes_reg **seq_out,
                    unsigned int *num_out)
{
    struct property *prop;
    struct max9296_serdes_reg *seq;
    u32 *raw_seq = NULL;
    int len;
    unsigned int i, count;
    int ret;

    prop = of_find_property(np, prop_name, &len);
    if (!prop) {
        *seq_out = NULL;
        *num_out = 0;
        return 0;
    }

    if (!len || len % (5 * sizeof(u32)))
        return -EINVAL;

    count = len / (5 * sizeof(u32));
    raw_seq = kmalloc(len, GFP_KERNEL);
    if (!raw_seq)
        return -ENOMEM;

    ret = of_property_read_u32_array(np, prop_name, raw_seq, len / sizeof(u32));
    if (ret)
        goto out;

    seq = devm_kcalloc(dev, count, sizeof(*seq), GFP_KERNEL);
    if (!seq) {
        ret = -ENOMEM;
        goto out;
    }

    for (i = 0; i < count; i++) {
        u32 *entry = &raw_seq[i * 5];

        seq[i].addr = entry[0];
        seq[i].page = entry[1];
        seq[i].reg = entry[2];
        seq[i].val = entry[3];
        seq[i].delay_ms = entry[4];
    }

    *seq_out = seq;
    *num_out = count;
    ret = 0;
out:
    kfree(raw_seq);
    return ret;
}

static int max9296_fixed_append_child_seq_by_compatible(
                    struct max9296_fixed_yuv *priv,
                    const char *compatible,
                    struct max9296_serdes_reg **seq_out,
                    unsigned int *count_out)
{
    struct device_node *child;
    struct max9296_serdes_reg *seq;
    unsigned int count;
    int ret;

    for_each_available_child_of_node(priv->client->dev.of_node, child) {
        if (!of_device_is_compatible(child, compatible))
            continue;

        dev_info(&priv->client->dev, "parse %s init sequence from %pOF\n",
             compatible, child);
        ret = max9296_fixed_parse_init_seq(&priv->client->dev, child,
                           "mixtile,init-sequence",
                           &seq, &count);
        if (ret)
            return ret;

        *seq_out = seq;
        *count_out = count;
        return 0;
    }

    *seq_out = NULL;
    *count_out = 0;
    dev_err(&priv->client->dev, "missing child node compatible %s\n",
        compatible);
    return -ENOENT;
}

    static inline struct max9296_fixed_yuv *to_max9296_fixed_yuv(struct v4l2_subdev *sd)
    {
        return container_of(sd, struct max9296_fixed_yuv, sd);
    }

    static void max9296_fixed_get_module_inf(struct max9296_fixed_yuv *priv,
                        struct rkmodule_inf *inf)
    {
        memset(inf, 0, sizeof(*inf));
        strscpy(inf->base.sensor, DRIVER_NAME, sizeof(inf->base.sensor));
        strscpy(inf->base.module, priv->module_name ?: DRIVER_NAME,
            sizeof(inf->base.module));
        strscpy(inf->base.lens, priv->lens_name ?: DRIVER_NAME,
            sizeof(inf->base.lens));
    }

    static long max9296_fixed_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
    {
        struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);

        switch (cmd) {
        case RKMODULE_GET_MODULE_INFO:
            max9296_fixed_get_module_inf(priv, arg);
            return 0;
        case RKMODULE_SET_QUICK_STREAM:
            return max9296_fixed_s_stream(sd, *((u32 *)arg));
        case RKMODULE_GET_VC_FMT_INFO: {
            struct rkmodule_vc_fmt_info *inf = arg;

            memset(inf, 0, sizeof(*inf));
            inf->width[1] = MAX9296_FIXED_WIDTH;
            inf->height[1] = MAX9296_FIXED_HEIGHT;
            inf->fps[1] = 30;
            return 0;
        }
        case RKMODULE_GET_START_STREAM_SEQ:
            *(int *)arg = RKMODULE_START_STREAM_FRONT;
            return 0;
        case RKCIF_CMD_SET_CSI_IDX:
            return 0;
        default:
            return -ENOIOCTLCMD;
        }
    }

    #ifdef CONFIG_COMPAT
    static long max9296_fixed_compat_ioctl32(struct v4l2_subdev *sd,
                        unsigned int cmd,
                        unsigned long arg)
    {
        void __user *up = compat_ptr(arg);
        struct rkmodule_inf *inf;
        long ret;

        switch (cmd) {
        case RKMODULE_GET_MODULE_INFO:
            inf = kzalloc(sizeof(*inf), GFP_KERNEL);
            if (!inf)
                return -ENOMEM;
            ret = max9296_fixed_ioctl(sd, cmd, inf);
            if (!ret)
                ret = copy_to_user(up, inf, sizeof(*inf));
            kfree(inf);
            return ret;
        default:
            return max9296_fixed_ioctl(sd, cmd, up);
        }
    }
    #endif

    static int max9296_fixed_s_power(struct v4l2_subdev *sd, int on)
    {
        return 0;
    }

    static int max9296_fixed_write_reg(struct max9296_fixed_yuv *priv,
                    u8 addr, u8 page, u8 reg, u8 val)
    {
        u8 buf[3] = { page, reg, val };
        struct i2c_msg msg = {
            .addr = addr,
            .flags = 0,
            .len = sizeof(buf),
            .buf = buf,
        };
        int ret;

        ret = i2c_transfer(priv->client->adapter, &msg, 1);
        if (ret == 1)
            return 0;
        if (ret >= 0)
            ret = -EIO;

        if (addr == 0x48) {
            msg.addr = 0x28;
            ret = i2c_transfer(priv->client->adapter, &msg, 1);
            if (ret == 1)
                return 0;
            if (ret >= 0)
                ret = -EIO;
        } else if (addr == 0x40) {
            msg.addr = 0x42;
            ret = i2c_transfer(priv->client->adapter, &msg, 1);
            if (ret == 1)
                return 0;
            if (ret >= 0)
                ret = -EIO;
        } else if (addr == 0x42) {
            msg.addr = 0x40;
            ret = i2c_transfer(priv->client->adapter, &msg, 1);
            if (ret == 1)
                return 0;
            if (ret >= 0)
                ret = -EIO;
        }

        dev_err(&priv->client->dev,
            "serdes write failed addr=0x%02x page=0x%02x reg=0x%02x val=0x%02x ret=%d\n",
            addr, page, reg, val, ret);
        return ret;
    }

static int max9296_fixed_run_seq(struct max9296_fixed_yuv *priv,
                 const char *name,
                 const struct max9296_serdes_reg *seq,
                 unsigned int count)
{
    unsigned int i;
    int ret;

    if (!count)
        return -EINVAL;

    dev_info(&priv->client->dev, "run %s init sequence (%u regs)\n",
         name, count);
    for (i = 0; i < count; i++) {
        const struct max9296_serdes_reg *reg = &seq[i];

        ret = max9296_fixed_write_reg(priv, reg->addr, reg->page,
                          reg->reg, reg->val);
        if (ret) {
            dev_err(&priv->client->dev,
                "%s init failed at step %u/%u addr=0x%02x page=0x%02x reg=0x%02x val=0x%02x\n",
                name, i + 1, count, reg->addr, reg->page,
                reg->reg, reg->val);
            return ret;
        }
        if (reg->delay_ms)
            msleep(reg->delay_ms);
    }

    dev_info(&priv->client->dev, "done %s init sequence\n", name);
    return 0;
}

static int max9296_fixed_apply_serdes_init(struct max9296_fixed_yuv *priv)
{
    int ret;

    if (!priv->max9296_seq_num || !priv->max96717_seq_num ||
        !priv->isx031_seq_num)
        return -EINVAL;

    dev_info(&priv->client->dev,
         "init sequences parsed: max9296=%u max96717=%u isx031=%u total=%u\n",
         priv->max9296_seq_num, priv->max96717_seq_num,
         priv->isx031_seq_num,
         priv->max9296_seq_num + priv->max96717_seq_num + priv->isx031_seq_num);

    ret = max9296_fixed_run_seq(priv, "max9296", priv->max9296_seq,
                    priv->max9296_seq_num);
    if (ret)
        return ret;
    ret = max9296_fixed_run_seq(priv, "max96717", priv->max96717_seq,
                    priv->max96717_seq_num);
    if (ret)
        return ret;
    ret = max9296_fixed_run_seq(priv, "isx031", priv->isx031_seq,
                    priv->isx031_seq_num);
    if (ret)
        return ret;

    priv->serdes_configured = true;
    return 0;
}

static int max9296_fixed_s_stream(struct v4l2_subdev *sd, int on)
{
    struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);

    mutex_lock(&priv->lock);
    priv->streaming = !!on;
    dev_info(&priv->client->dev,
         "stream %s fixed MAX9296 YUV source %ux%u (serdes %s)\n",
         on ? "on" : "off", priv->fmt.width, priv->fmt.height,
         priv->serdes_configured ? "ready" : "not-ready");
    mutex_unlock(&priv->lock);

    return 0;
}

    static int max9296_fixed_g_frame_interval(struct v4l2_subdev *sd,
                        struct v4l2_subdev_frame_interval *fi)
    {
        struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);

        mutex_lock(&priv->lock);
        fi->interval = priv->frame_interval;
        mutex_unlock(&priv->lock);

        return 0;
    }

    static int max9296_fixed_enum_mbus_code(struct v4l2_subdev *sd,
                        struct v4l2_subdev_state *sd_state,
                        struct v4l2_subdev_mbus_code_enum *code)
    {
        if (code->index)
            return -EINVAL;

        code->code = MEDIA_BUS_FMT_UYVY8_2X8;
        return 0;
    }

    static int max9296_fixed_enum_frame_size(struct v4l2_subdev *sd,
                        struct v4l2_subdev_state *sd_state,
                        struct v4l2_subdev_frame_size_enum *fse)
    {
        if (fse->index || fse->code != MEDIA_BUS_FMT_UYVY8_2X8)
            return -EINVAL;

        fse->min_width = MAX9296_FIXED_WIDTH;
        fse->max_width = MAX9296_FIXED_WIDTH;
        fse->min_height = MAX9296_FIXED_HEIGHT;
        fse->max_height = MAX9296_FIXED_HEIGHT;
        return 0;
    }

    static int max9296_fixed_enum_frame_interval(
        struct v4l2_subdev *sd,
        struct v4l2_subdev_state *sd_state,
        struct v4l2_subdev_frame_interval_enum *fie)
    {
        if (fie->index || fie->code != MEDIA_BUS_FMT_UYVY8_2X8 ||
            fie->width != MAX9296_FIXED_WIDTH ||
            fie->height != MAX9296_FIXED_HEIGHT)
            return -EINVAL;

        fie->interval.numerator = 1;
        fie->interval.denominator = 30;
        return 0;
    }

    static int max9296_fixed_get_fmt(struct v4l2_subdev *sd,
                    struct v4l2_subdev_state *sd_state,
                    struct v4l2_subdev_format *fmt)
    {
        struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);

        mutex_lock(&priv->lock);
        fmt->format = priv->fmt;
        fmt->format.reserved[0] = 1;
        mutex_unlock(&priv->lock);
        return 0;
    }

    static int max9296_fixed_set_fmt(struct v4l2_subdev *sd,
                    struct v4l2_subdev_state *sd_state,
                    struct v4l2_subdev_format *fmt)
    {
        struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);

        mutex_lock(&priv->lock);
        fmt->format = priv->fmt;
        fmt->format.reserved[0] = 1;
        mutex_unlock(&priv->lock);
        return 0;
    }

    static int max9296_fixed_get_mbus_config(struct v4l2_subdev *sd,
                        unsigned int pad,
                        struct v4l2_mbus_config *config)
    {
        struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);
        u8 data_lanes;

        config->type = V4L2_MBUS_CSI2_DPHY;
        data_lanes = priv->bus_cfg.bus.mipi_csi2.num_data_lanes ?
                priv->bus_cfg.bus.mipi_csi2.num_data_lanes : 2;
        config->bus.mipi_csi2.num_data_lanes = data_lanes;
        config->bus.mipi_csi2.flags = priv->bus_cfg.bus.mipi_csi2.flags;
        config->bus.mipi_csi2.clock_lane = priv->bus_cfg.bus.mipi_csi2.clock_lane;
        return 0;
    }

    #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
    static int max9296_fixed_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
    {
        struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);
        struct v4l2_mbus_framefmt *try_fmt =
            v4l2_subdev_get_try_format(sd, fh->state, 0);

        mutex_lock(&priv->lock);
        *try_fmt = priv->fmt;
        try_fmt->reserved[0] = 1;
        mutex_unlock(&priv->lock);
        return 0;
    }

    static const struct v4l2_subdev_internal_ops max9296_fixed_internal_ops = {
        .open = max9296_fixed_open,
    };
    #endif

    static const struct v4l2_subdev_core_ops max9296_fixed_core_ops = {
        .s_power = max9296_fixed_s_power,
        .ioctl = max9296_fixed_ioctl,
    #ifdef CONFIG_COMPAT
        .compat_ioctl32 = max9296_fixed_compat_ioctl32,
    #endif
    };

    static const struct v4l2_subdev_video_ops max9296_fixed_video_ops = {
        .s_stream = max9296_fixed_s_stream,
        .g_frame_interval = max9296_fixed_g_frame_interval,
    };

    static const struct v4l2_subdev_pad_ops max9296_fixed_pad_ops = {
        .enum_mbus_code = max9296_fixed_enum_mbus_code,
        .enum_frame_size = max9296_fixed_enum_frame_size,
        .enum_frame_interval = max9296_fixed_enum_frame_interval,
        .get_fmt = max9296_fixed_get_fmt,
        .set_fmt = max9296_fixed_set_fmt,
        .get_mbus_config = max9296_fixed_get_mbus_config,
    };

    static const struct v4l2_subdev_ops max9296_fixed_subdev_ops = {
        .core = &max9296_fixed_core_ops,
        .video = &max9296_fixed_video_ops,
        .pad = &max9296_fixed_pad_ops,
    };

    static int max9296_fixed_init_controls(struct max9296_fixed_yuv *priv)
    {
        int ret;

        ret = v4l2_ctrl_handler_init(&priv->ctrl_handler, 2);
        if (ret)
            return ret;

        priv->link_freq = v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL,
                            V4L2_CID_LINK_FREQ,
                            0, 0,
                            max9296_fixed_link_freq_menu);
        priv->pixel_rate = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL,
                            V4L2_CID_PIXEL_RATE, 0,
                            MAX9296_FIXED_PIXEL_RATE, 1,
                            MAX9296_FIXED_PIXEL_RATE);
        if (priv->ctrl_handler.error)
            return priv->ctrl_handler.error;

        priv->sd.ctrl_handler = &priv->ctrl_handler;
        __v4l2_ctrl_s_ctrl(priv->link_freq, 0);
        __v4l2_ctrl_s_ctrl_int64(priv->pixel_rate, MAX9296_FIXED_PIXEL_RATE);
        return 0;
    }

static int max9296_fixed_parse_of(struct max9296_fixed_yuv *priv)
{
    struct device *dev = &priv->client->dev;
    struct device_node *endpoint;
    struct max9296_serdes_reg *seq;
    unsigned int count;
    int ret;

        endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
        if (!endpoint)
            return -EINVAL;

        ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint), &priv->bus_cfg);
        of_node_put(endpoint);
        if (ret)
            return ret;
        if (priv->bus_cfg.bus_type != V4L2_MBUS_CSI2_DPHY)
            return -EINVAL;

        of_property_read_u32(dev->of_node, "rockchip,camera-module-index",
                    &priv->module_index);
    of_property_read_string(dev->of_node, "rockchip,camera-module-facing",
                &priv->module_facing);
    of_property_read_string(dev->of_node, "rockchip,camera-module-name",
                &priv->module_name);
    of_property_read_string(dev->of_node, "rockchip,camera-module-lens-name",
                &priv->lens_name);

    ret = max9296_fixed_parse_init_seq(dev, dev->of_node,
                       "mixtile,max9296-init-sequence",
                       &seq, &count);
    if (ret)
        return ret;

    priv->max9296_seq_num = count;
    priv->max9296_seq = seq;
    dev_info(dev, "parse max9296 init sequence from %pOF (%u regs)\n",
         dev->of_node, priv->max9296_seq_num);

    ret = max9296_fixed_append_child_seq_by_compatible(priv,
                    "mixtile,max96717-script",
                    &priv->max96717_seq,
                    &priv->max96717_seq_num);
    if (ret)
        return ret;

    ret = max9296_fixed_append_child_seq_by_compatible(priv,
                    "sony,isx031-script",
                    &priv->isx031_seq,
                    &priv->isx031_seq_num);
    if (ret)
        return ret;

    return 0;
}

    static int max9296_fixed_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
    {
        struct max9296_fixed_yuv *priv;
        struct v4l2_subdev *sd;
        char facing[2] = "b";
        int ret;

        priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
        if (!priv)
            return -ENOMEM;

        priv->client = client;
        mutex_init(&priv->lock);

        ret = max9296_fixed_parse_of(priv);
        if (ret)
            goto err_mutex;

        priv->fmt.width = MAX9296_FIXED_WIDTH;
        priv->fmt.height = MAX9296_FIXED_HEIGHT;
        priv->fmt.code = MEDIA_BUS_FMT_UYVY8_2X8;
        priv->fmt.field = V4L2_FIELD_NONE;
        priv->fmt.colorspace = V4L2_COLORSPACE_SMPTE170M;
        priv->fmt.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
        priv->fmt.quantization = V4L2_QUANTIZATION_DEFAULT;
        priv->fmt.xfer_func = V4L2_XFER_FUNC_DEFAULT;
        priv->fmt.reserved[0] = 1;
        priv->frame_interval.numerator = 1;
        priv->frame_interval.denominator = 30;

        sd = &priv->sd;
        v4l2_i2c_subdev_init(sd, client, &max9296_fixed_subdev_ops);
        sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
        sd->internal_ops = &max9296_fixed_internal_ops;
    #endif

        priv->pad.flags = MEDIA_PAD_FL_SOURCE;
        sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
        ret = media_entity_pads_init(&sd->entity, 1, &priv->pad);
        if (ret)
            goto err_mutex;

        ret = max9296_fixed_init_controls(priv);
        if (ret)
            goto err_entity;

        ret = max9296_fixed_apply_serdes_init(priv);
        if (ret)
            goto err_ctrls;

        if (priv->module_facing && !strcmp(priv->module_facing, "front"))
            facing[0] = 'f';
        snprintf(sd->name, sizeof(sd->name), "m%02u_%s_%s %s",
            priv->module_index, facing, DRIVER_NAME, dev_name(sd->dev));

        ret = v4l2_async_register_subdev_sensor(sd);
        if (ret)
            goto err_ctrls;

        dev_info(&client->dev, "registered fixed MAX9296 YUV source bridge\n");
        return 0;

    err_ctrls:
        v4l2_ctrl_handler_free(&priv->ctrl_handler);
    err_entity:
        media_entity_cleanup(&sd->entity);
    err_mutex:
        mutex_destroy(&priv->lock);
        return ret;
    }

    static void max9296_fixed_remove(struct i2c_client *client)
    {
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct max9296_fixed_yuv *priv = to_max9296_fixed_yuv(sd);

        v4l2_async_unregister_subdev(sd);
        v4l2_ctrl_handler_free(&priv->ctrl_handler);
        media_entity_cleanup(&sd->entity);
        mutex_destroy(&priv->lock);
    }

    static const struct i2c_device_id max9296_fixed_id[] = {
        { DRIVER_NAME, 0 },
        { }
    };
    MODULE_DEVICE_TABLE(i2c, max9296_fixed_id);

    static const struct of_device_id max9296_fixed_of_match[] = {
        { .compatible = "mixtile,max9296-fixed-yuv" },
        { }
    };
    MODULE_DEVICE_TABLE(of, max9296_fixed_of_match);

    static struct i2c_driver max9296_fixed_i2c_driver = {
        .driver = {
            .name = DRIVER_NAME,
            .of_match_table = of_match_ptr(max9296_fixed_of_match),
        },
        .probe = max9296_fixed_probe,
        .remove = max9296_fixed_remove,
        .id_table = max9296_fixed_id,
    };

    module_i2c_driver(max9296_fixed_i2c_driver);

    MODULE_DESCRIPTION("Fixed MAX9296 YUV source bridge");
    MODULE_LICENSE("GPL");
